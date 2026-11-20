/* ============================================================
 * HAL IPC — IMPLEMENTATION
 * ============================================================ */

#include "hal_ipc.h"
#include "hal_time.h"
#include "hal_cache.h"
#include "error_handler.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/* STM32 HAL for HSEM */
#include "stm32h7xx_hal.h"

#include <string.h>

/* ============================================================
 * Shared Memory Layout
 * ============================================================ */
#define IPC_MAGIC               0xCAFEBABEUL
#define IPC_VERSION             1U
#define IPC_RING_ENTRIES        8U
#define IPC_ENTRY_MAX_BYTES     (sizeof(ipc_ai_msg_t))

/* HSEM IDs (must match cm4_ipc.h) */
#define HSEM_CH_CM7_TO_CM4      0U
#define HSEM_CH_CM4_TO_CM7      1U
#define HSEM_STATUS             2U

#define HSEM_TIMEOUT_CYCLES     (CFG_CPU_FREQ_HZ / 1000U)   /* 1 ms */

/* ---------- Ring Entry ---------- */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t            msg_type;
    uint32_t            sequence_id;
    uint32_t            size;
    uint32_t            crc32;
    uint8_t             payload[IPC_ENTRY_MAX_BYTES];
    uint32_t            reserved[2];
} ipc_entry_t;

/* ---------- Ring ---------- */
typedef struct __attribute__((packed, aligned(4))) {
    volatile uint32_t   head;
    volatile uint32_t   tail;
    volatile uint32_t   count;
    uint32_t            overflow_count;
    uint32_t            reserved[4];
    ipc_entry_t         entries[IPC_RING_ENTRIES];
} ipc_ring_t;

/* ---------- Shared Structure ---------- */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t            magic;
    uint32_t            version;
    uint32_t            reserved_header[6];
    ipc_ring_t          cm7_to_cm4;
    ipc_ring_t          cm4_to_cm7;
    ipc_status_t        status;
    uint32_t            reserved_tail[16];
} ipc_shared_t;

/* ---------- Shared Memory Address ---------- */
static ipc_shared_t * const g_shared = (ipc_shared_t *)CFG_IPC_SHARED_ADDR;

/* ---------- Internal State ---------- */
static bool g_initialized = false;
static uint32_t g_seq_counter = 0U;

/* ============================================================
 * CRC32 (bitwise, small footprint)
 * ============================================================ */
static uint32_t crc32(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return 0U;
    }

    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (uint32_t b = 0U; b < 8U; b++) {
            if (crc & 1U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ============================================================
 * HSEM lock / unlock — uses DWT cycles for bounded wait
 * ============================================================ */
proj_status_t hal_ipc_lock(uint32_t hsem_id)
{
    uint32_t start = DWT->CYCCNT;

    while (HAL_HSEM_FastTake(hsem_id) != HAL_OK) {
        uint32_t elapsed = DWT->CYCCNT - start;
        if (elapsed > HSEM_TIMEOUT_CYCLES) {
            return PROJ_ERR_TIMEOUT;
        }
    }
    return PROJ_OK;
}

proj_status_t hal_ipc_unlock(uint32_t hsem_id)
{
    HAL_HSEM_Release(hsem_id, 0U);
    return PROJ_OK;
}

/* ============================================================
 * Internal: HSEM ID for channel
 * ============================================================ */
static uint32_t hsem_for_channel(hal_ipc_channel_t ch)
{
    return (ch == HAL_IPC_CH_CM7_TO_CM4) ? HSEM_CH_CM7_TO_CM4
                                         : HSEM_CH_CM4_TO_CM7;
}

/* ============================================================
 * Internal: Ring for channel
 * ============================================================ */
static ipc_ring_t *get_ring(hal_ipc_channel_t ch)
{
    return (ch == HAL_IPC_CH_CM7_TO_CM4) ? &g_shared->cm7_to_cm4
                                         : &g_shared->cm4_to_cm7;
}

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_ipc_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    __HAL_RCC_HSEM_CLK_ENABLE();

    /* Only CM7 initializes shared region */
    g_shared->magic   = IPC_MAGIC;
    g_shared->version = IPC_VERSION;

    memset(&g_shared->cm7_to_cm4, 0, sizeof(ipc_ring_t));
    memset(&g_shared->cm4_to_cm7, 0, sizeof(ipc_ring_t));
    memset(&g_shared->status,     0, sizeof(ipc_status_t));

    g_seq_counter = 0U;
    g_initialized = true;

    return PROJ_OK;
}

/* ============================================================
 * SEND
 * ============================================================ */
proj_status_t hal_ipc_send(hal_ipc_channel_t ch,
                           const void        *data,
                           size_t             size,
                           uint32_t           timeout_us)
{
    VALIDATE_NOT_NULL(data);
    VALIDATE_RETURN(size == 0U, PROJ_ERR_INVALID);
    VALIDATE_RETURN(size > IPC_ENTRY_MAX_BYTES, PROJ_ERR_OVERFLOW);
    VALIDATE_RETURN(ch >= HAL_IPC_CH_COUNT, PROJ_ERR_INVALID);

    if (!g_initialized) {
        return PROJ_ERR_NOT_INIT;
    }

    ipc_ring_t *ring = get_ring(ch);
    uint32_t    hsem = hsem_for_channel(ch);

    uint32_t start_us = hal_time_us_32();

    /* Acquire HSEM with retry */
    while (hal_ipc_lock(hsem) != PROJ_OK) {
        if (hal_timeout_elapsed_us(start_us, timeout_us)) {
            return PROJ_ERR_TIMEOUT;
        }
        k_yield();
    }

    proj_status_t ret = PROJ_OK;

    if (ring->count >= IPC_RING_ENTRIES) {
        ring->overflow_count++;
        ret = PROJ_ERR_BUSY;
    } else {
        uint32_t idx = ring->head;
        ipc_entry_t *e = &ring->entries[idx];

        e->msg_type    = 0U;
        e->sequence_id = g_seq_counter++;
        e->size        = (uint32_t)size;
        e->crc32       = crc32((const uint8_t *)data, size);
        memcpy(e->payload, data, size);

        /* Flush D-Cache: peer core must see fresh data */
        hal_dcache_flush(e, sizeof(ipc_entry_t));

        ring->head = (idx + 1U) % IPC_RING_ENTRIES;
        ring->count++;
    }

    (void)hal_ipc_unlock(hsem);

    return ret;
}

/* ============================================================
 * RECEIVE
 * ============================================================ */
proj_status_t hal_ipc_recv(hal_ipc_channel_t ch,
                           void              *buf,
                           size_t             buf_size,
                           size_t            *received_size,
                           uint32_t           timeout_us)
{
    VALIDATE_NOT_NULL(buf);
    VALIDATE_NOT_NULL(received_size);
    VALIDATE_RETURN(ch >= HAL_IPC_CH_COUNT, PROJ_ERR_INVALID);

    if (!g_initialized) {
        return PROJ_ERR_NOT_INIT;
    }

    ipc_ring_t *ring = get_ring(ch);
    uint32_t    hsem = hsem_for_channel(ch);

    uint32_t start_us = hal_time_us_32();

    while (hal_ipc_lock(hsem) != PROJ_OK) {
        if (hal_timeout_elapsed_us(start_us, timeout_us)) {
            return PROJ_ERR_TIMEOUT;
        }
        k_yield();
    }

    proj_status_t ret = PROJ_OK;

    if (ring->count == 0U) {
        ret = PROJ_ERR_NOT_FOUND;
    } else {
        uint32_t idx = ring->tail;
        ipc_entry_t *e = &ring->entries[idx];

        /* Invalidate D-Cache: fetch fresh copy from memory */
        hal_dcache_invalidate(e, sizeof(ipc_entry_t));

        if (e->size > buf_size || e->size > IPC_ENTRY_MAX_BYTES) {
            ret = PROJ_ERR_OVERFLOW;
        } else {
            uint32_t computed_crc = crc32(e->payload, e->size);
            if (computed_crc != e->crc32) {
                ret = PROJ_ERR_CHECKSUM;
            } else {
                memcpy(buf, e->payload, e->size);
                *received_size = e->size;
            }
        }

        ring->tail = (idx + 1U) % IPC_RING_ENTRIES;
        ring->count--;
    }

    (void)hal_ipc_unlock(hsem);

    return ret;
}

/* ============================================================
 * STATUS
 * ============================================================ */
uint32_t hal_ipc_pending(hal_ipc_channel_t ch)
{
    if (ch >= HAL_IPC_CH_COUNT || !g_initialized) {
        return 0U;
    }
    return get_ring(ch)->count;
}

uint32_t hal_ipc_overflow_count(hal_ipc_channel_t ch)
{
    if (ch >= HAL_IPC_CH_COUNT || !g_initialized) {
        return 0U;
    }
    return get_ring(ch)->overflow_count;
}

bool hal_ipc_peer_alive(void)
{
    if (!g_initialized) {
        return false;
    }
    return (g_shared->status.cm4_alive != 0U);
}

const ipc_status_t *hal_ipc_status(void)
{
    return &g_shared->status;
}