#include "../Inc/cm4_ipc.h"

/* Zephyr HAL abstraction */
#include "hal/hal_time.h"
#include "hal/hal_ipc.h"

/* ============================================================
 * CM4 IPC — Implementation (shared between CM7 and CM4)
 * ============================================================
 * Both cores include this file. Compile-time:
 *   CM7: -DCORE_CM7  -> uses HSEM from CM7 side
 *   CM4: -DCORE_CM4  -> uses HSEM from CM4 side
 *
 * SAFETY: HSEM lock uses bounded wait (1 ms timeout) inside
 *         hal_ipc_lock(). No infinite-loop deadlock.
 * ============================================================ */

/* ---------- CRC32 (bitwise) ---------- */
uint32_t cm4_ipc_crc32(const uint8_t *data, uint32_t len)
{
    if (data == NULL) return 0U;

    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (uint32_t b = 0U; b < 8U; b++) {
            if (crc & 1U) crc = (crc >> 1U) ^ 0xEDB88320U;
            else          crc >>= 1U;
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ---------- HSEM lock (via HAL abstraction) ---------- */
static inline bool hsem_lock(uint32_t id)
{
    return (hal_ipc_lock(id) == PROJ_OK);
}

/* ---------- HSEM release ---------- */
static inline void hsem_unlock(uint32_t id)
{
    (void)hal_ipc_unlock(id);
}

/* ---------- IPCC notify (polling mode) ---------- */
static inline void ipcc_notify_peer(void)
{
    /* IPCC not enabled. Polling mode. */
}

/* ---------- INIT ---------- */
bool cm4_ipc_init(void)
{
    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

#if defined(CORE_CM7)
    /* Only CM7 initializes the shared region */
    sh->magic   = CM4_IPC_MAGIC;
    sh->version = CM4_IPC_VERSION;

    sh->cm7_to_cm4.head = 0U;
    sh->cm7_to_cm4.tail = 0U;
    sh->cm7_to_cm4.count = 0U;
    sh->cm7_to_cm4.overflow_count = 0U;

    sh->cm4_to_cm7.head = 0U;
    sh->cm4_to_cm7.tail = 0U;
    sh->cm4_to_cm7.count = 0U;
    sh->cm4_to_cm7.overflow_count = 0U;

    sh->status.cm4_alive = 0U;
    sh->status.cm4_last_heartbeat_us = 0U;
    sh->status.cm4_ai_inference_count = 0U;
    sh->status.cm4_ai_latency_us = 0U;
    sh->status.cm4_ai_latency_max_us = 0U;
    sh->status.cm4_fault_count = 0U;

    return true;
#elif defined(CORE_CM4)
    /* CM4 waits for CM7 to initialize */
    for (uint32_t i = 0U; i < 1000000U; i++) {
        if (sh->magic == CM4_IPC_MAGIC) return true;
    }
    return false;
#else
    return false;
#endif
}

bool cm4_ipc_validate(void)
{
    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;
    return (sh->magic == CM4_IPC_MAGIC) && (sh->version == CM4_IPC_VERSION);
}

/* ---------- DEBUG: get magic ---------- */
uint32_t cm4_ipc_get_magic(void)
{
    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;
    return sh->magic;
}

/* ---------- Internal ring push (single producer) ---------- */
static bool ring_push(cm4_ipc_ring_t *r, const cm4_ipc_entry_t *e)
{
    if (r->count >= CM4_IPC_RING_SIZE) {
        r->overflow_count++;
        return false;
    }
    uint32_t idx = r->head;
    r->entries[idx] = *e;
    r->head = (idx + 1U) % CM4_IPC_RING_SIZE;
    r->count++;
    return true;
}

/* ---------- Internal ring pop (single consumer) ---------- */
static bool ring_pop(cm4_ipc_ring_t *r, cm4_ipc_entry_t *e)
{
    if (r->count == 0U) return false;
    uint32_t idx = r->tail;
    *e = r->entries[idx];
    r->tail = (idx + 1U) % CM4_IPC_RING_SIZE;
    r->count--;
    return true;
}

/* ---------- SEND input (CM7 -> CM4) ---------- */
bool cm4_ipc_send_input(const cm4_ipc_ai_msg_t *msg)
{
    if (msg == NULL) return false;

    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_RING_CM7_TO_CM4)) {
        return false;
    }

    cm4_ipc_entry_t e;
    e.msg_type    = CM4_IPC_MSG_AI_INPUT;
    e.sequence_id = msg->sequence_id;

    e.payload = *msg;
    e.payload.crc32 = cm4_ipc_crc32((const uint8_t *)msg->data, CM4_IPC_AI_BYTES);

    bool ok = ring_push(&sh->cm7_to_cm4, &e);

    hsem_unlock(CM4_HSEM_RING_CM7_TO_CM4);

    if (ok) ipcc_notify_peer();
    return ok;
}

/* ---------- RECV input (CM4 side) ---------- */
bool cm4_ipc_recv_input(cm4_ipc_ai_msg_t *msg)
{
    if (msg == NULL) return false;

    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_RING_CM7_TO_CM4)) {
        return false;
    }

    cm4_ipc_entry_t e;
    bool ok = ring_pop(&sh->cm7_to_cm4, &e);

    hsem_unlock(CM4_HSEM_RING_CM7_TO_CM4);

    if (!ok) return false;

    uint32_t crc = cm4_ipc_crc32((const uint8_t *)e.payload.data, CM4_IPC_AI_BYTES);
    if (crc != e.payload.crc32) return false;

    *msg = e.payload;
    return true;
}

/* ---------- SEND output (CM4 -> CM7) ---------- */
bool cm4_ipc_send_output(const cm4_ipc_ai_msg_t *msg)
{
    if (msg == NULL) return false;

    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_RING_CM4_TO_CM7)) {
        return false;
    }

    cm4_ipc_entry_t e;
    e.msg_type    = CM4_IPC_MSG_AI_OUTPUT;
    e.sequence_id = msg->sequence_id;

    e.payload = *msg;
    e.payload.crc32 = cm4_ipc_crc32((const uint8_t *)msg->data, CM4_IPC_AI_BYTES);

    bool ok = ring_push(&sh->cm4_to_cm7, &e);

    hsem_unlock(CM4_HSEM_RING_CM4_TO_CM7);

    if (ok) ipcc_notify_peer();
    return ok;
}

/* ---------- RECV output (CM7 side) ---------- */
bool cm4_ipc_recv_output(cm4_ipc_ai_msg_t *msg)
{
    if (msg == NULL) return false;

    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_RING_CM4_TO_CM7)) {
        return false;
    }

    cm4_ipc_entry_t e;
    bool ok = ring_pop(&sh->cm4_to_cm7, &e);

    hsem_unlock(CM4_HSEM_RING_CM4_TO_CM7);

    if (!ok) return false;

    uint32_t crc = cm4_ipc_crc32((const uint8_t *)e.payload.data, CM4_IPC_AI_BYTES);
    if (crc != e.payload.crc32) return false;

    *msg = e.payload;
    return true;
}

/* ---------- STATUS UPDATE (CM4 only) ---------- */
bool cm4_ipc_status_update(void)
{
#if defined(CORE_CM4)
    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_STATUS)) {
        return false;
    }
    sh->status.cm4_alive = 1U;
    sh->status.cm4_last_heartbeat_us = hal_time_ms_32();
    hsem_unlock(CM4_HSEM_STATUS);
    return true;
#else
    return false;
#endif
}

/* ---------- STATUS READ (CM7 only) ---------- */
bool cm4_ipc_status_read(cm4_ipc_status_t *out)
{
    if (out == NULL) return false;

    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_STATUS)) {
        return false;
    }
    *out = sh->status;
    hsem_unlock(CM4_HSEM_STATUS);

    return true;
}

/* ---------- AI INFERENCE COUNT UPDATE (CM4 only) ---------- */
void cm4_ipc_inference_count_inc(uint32_t latency_us)
{
#if defined(CORE_CM4)
    cm4_ipc_shared_t *sh = CM4_IPC_SHARED_ADDR;

    if (!hsem_lock(CM4_HSEM_STATUS)) {
        return;
    }

    sh->status.cm4_ai_inference_count++;
    sh->status.cm4_ai_latency_us = latency_us;
    if (latency_us > sh->status.cm4_ai_latency_max_us) {
        sh->status.cm4_ai_latency_max_us = latency_us;
    }

    hsem_unlock(CM4_HSEM_STATUS);
#else
    (void)latency_us;
#endif
}