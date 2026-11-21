#include "agent_ipc.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT IPC — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per send/receive
 * ============================================================ */

/* PERF: state in DTCM (zero wait state, single-writer per ring) */
static AGENT_DTCM agent_ipc_state_t g_ipc;
static bool                         g_ipc_initialized = false;

/* ---------- Ring push (producer) ---------- */
AGENT_INLINE_FAST agent_status_t ring_push(agent_ipc_ring_t *r, const agent_ipc_msg_t *m)
{
    if (r->count >= AGENT_IPC_RING_SIZE) {
        r->drop_count++;
        return AGENT_ERR;
    }

    uint32_t idx = r->head;
    r->ring[idx] = *m;

    r->head = (idx + 1U) % AGENT_IPC_RING_SIZE;
    r->count++;

    return AGENT_OK;
}

/* ---------- Ring pop (consumer) ---------- */
AGENT_INLINE_FAST agent_status_t ring_pop(agent_ipc_ring_t *r, agent_ipc_msg_t *m)
{
    if (r->count == 0U) {
        return AGENT_ERR;
    }

    uint32_t idx = r->tail;
    *m = r->ring[idx];

    r->tail = (idx + 1U) % AGENT_IPC_RING_SIZE;
    r->count--;

    return AGENT_OK;
}

/* ---------- INIT ---------- */
agent_status_t agent_ipc_init(void)
{
    memset(&g_ipc, 0, sizeof(g_ipc));

    g_ipc.tx.head       = 0U;
    g_ipc.tx.tail       = 0U;
    g_ipc.tx.count      = 0U;
    g_ipc.tx.drop_count = 0U;

    g_ipc.rx.head       = 0U;
    g_ipc.rx.tail       = 0U;
    g_ipc.rx.count      = 0U;
    g_ipc.rx.drop_count = 0U;

    g_ipc.last_heartbeat_us = agent_time_now_us();
    g_ipc_initialized = true;

    return AGENT_OK;
}

agent_status_t agent_ipc_reset(void)
{
    g_ipc_initialized = false;
    return agent_ipc_init();
}

/* ---------- SEND ---------- */
agent_status_t agent_ipc_send(uint32_t msg_id, uint32_t arg1, uint32_t arg2)
{
    if (!g_ipc_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_ipc_msg_t m;
    memset(&m, 0, sizeof(m));

    m.msg_id       = msg_id;
    m.arg1         = arg1;
    m.arg2         = arg2;
    m.timestamp_us = agent_time_now_us();
    m.payload_size = 0U;

    agent_status_t st = ring_push(&g_ipc.tx, &m);

    if (st == AGENT_OK) {
        g_ipc.tx_total++;
        g_ipc.tx.last_tx_us = m.timestamp_us;
    } else {
        g_ipc.tx_errors++;
        agent_log_event(AGENT_LOG_IPC_TIMEOUT, msg_id, g_ipc.tx.drop_count);
    }

    return st;
}

/* ---------- SEND PAYLOAD ---------- */
agent_status_t agent_ipc_send_payload(uint32_t       msg_id,
                                      const uint8_t *payload,
                                      uint8_t        payload_size)
{
    if (!g_ipc_initialized)                                 return AGENT_ERR_NOT_INIT;
    if (payload == NULL && payload_size > 0U)               return AGENT_ERR;
    if (payload_size > AGENT_IPC_MAX_PAYLOAD)               return AGENT_ERR;

    agent_ipc_msg_t m;
    memset(&m, 0, sizeof(m));

    m.msg_id       = msg_id;
    m.arg1         = 0U;
    m.arg2         = 0U;
    m.timestamp_us = agent_time_now_us();
    m.payload_size = payload_size;

    if (payload_size > 0U) {
        memcpy(m.payload, payload, payload_size);
    }

    agent_status_t st = ring_push(&g_ipc.tx, &m);

    if (st == AGENT_OK) {
        g_ipc.tx_total++;
        g_ipc.tx.last_tx_us = m.timestamp_us;
    } else {
        g_ipc.tx_errors++;
    }

    return st;
}

/* ---------- RECEIVE ---------- */
agent_status_t agent_ipc_receive(agent_ipc_msg_t *out)
{
    if (out == NULL)        return AGENT_ERR;
    if (!g_ipc_initialized) return AGENT_ERR_NOT_INIT;

    agent_status_t st = ring_pop(&g_ipc.rx, out);

    if (st == AGENT_OK) {
        g_ipc.rx_total++;
        g_ipc.rx.last_rx_us = agent_time_now_us();
    } else {
        g_ipc.rx_errors++;
    }

    return st;
}

/* ---------- HAS MESSAGE ---------- */
bool agent_ipc_has_message(void)
{
    return (g_ipc.rx.count > 0U);
}

/* ---------- HEARTBEAT ---------- */
agent_status_t agent_ipc_heartbeat(void)
{
    if (!g_ipc_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    uint32_t now = agent_time_now_us();
    g_ipc.last_heartbeat_us = now;

    return agent_ipc_send(AGENT_IPC_MSG_HEARTBEAT, now, 0U);
}

/* ---------- IS ALIVE ---------- */
bool agent_ipc_is_alive(void)
{
    uint32_t now = agent_time_now_us();
    uint32_t dt  = now - g_ipc.last_heartbeat_us;

    return (dt <= (AGENT_IPC_HEARTBEAT_US * 5U)) ? true : false;
}

/* ---------- UPDATE ---------- */
agent_status_t agent_ipc_update(void)
{
    if (!g_ipc_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    uint32_t now = agent_time_now_us();

    /* Heartbeat timeout tracking */
    if ((now - g_ipc.last_heartbeat_us) > AGENT_IPC_TIMEOUT_US) {
        g_ipc.timeout_count++;
        agent_log_event(AGENT_LOG_IPC_TIMEOUT, 0U, g_ipc.timeout_count);
    }

    /* Drain incoming messages */
    while (agent_ipc_has_message()) {

        agent_ipc_msg_t m;
        if (agent_ipc_receive(&m) != AGENT_OK) {
            break;
        }

        switch (m.msg_id) {
        case AGENT_IPC_MSG_HEARTBEAT:
            g_ipc.last_heartbeat_us = now;
            break;

        case AGENT_IPC_MSG_EMERGENCY:
            agent_log_event(AGENT_LOG_EMERGENCY, m.arg1, m.arg2);
            break;

        case AGENT_IPC_MSG_FAULT_REPORT:
            agent_log_event(AGENT_LOG_IPC_RECV, m.arg1, m.arg2);
            break;

        default:
            break;
        }
    }

    return AGENT_OK;
}

/* ---------- STATE ---------- */
const agent_ipc_state_t *agent_ipc_state(void)
{
    return &g_ipc;
}

/* ---------- TIMEOUT HOOK ---------- */
void agent_ipc_on_timeout(void)
{
    g_ipc.timeout_count++;
    agent_log_event(AGENT_LOG_IPC_TIMEOUT, 0xFFFFU, g_ipc.timeout_count);
}