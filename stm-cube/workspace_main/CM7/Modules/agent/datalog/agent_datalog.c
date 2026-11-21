#include "agent_datalog.h"
#include "agent_datalog_ring.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT DATALOG — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per write
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_datalog_state_t g_dlog;
static bool                             g_dlog_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_datalog_init(void)
{
    memset(&g_dlog, 0, sizeof(g_dlog));   /* zero-init incl. reserved[] */

    g_dlog.head          = 0U;
    g_dlog.tail          = 0U;
    g_dlog.count         = 0U;
    g_dlog.total_written = 0U;
    g_dlog.total_dropped = 0U;
    g_dlog.last_write_us = 0U;
    g_dlog.enabled       = true;

    agent_datalog_ring_init();

    g_dlog_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_datalog_reset(void)
{
    g_dlog_initialized = false;
    return agent_datalog_init();
}

/* ---------- ENABLE ---------- */
agent_status_t agent_datalog_enable(bool en)
{
    g_dlog.enabled = en;
    return AGENT_OK;
}

/* ---------- WRITE (no payload) ---------- */
agent_status_t agent_datalog_write(agent_datalog_type_t type,
                                   uint32_t             arg1,
                                   uint32_t             arg2)
{
    return agent_datalog_write_payload(type, NULL, 0U);
}

/* ---------- WRITE PAYLOAD ---------- */
agent_status_t agent_datalog_write_payload(agent_datalog_type_t type,
                                           const uint8_t       *payload,
                                           uint8_t              payload_size)
{
    if (!g_dlog_initialized || !g_dlog.enabled)         return AGENT_ERR_NOT_INIT;
    if (payload_size > AGENT_DATALOG_ENTRY_MAX_PAYLOAD) return AGENT_ERR;

    /* Ring overflow → drop + count */
    if (g_dlog.count >= AGENT_DATALOG_RING_SIZE) {
        g_dlog.total_dropped++;
        return AGENT_ERR;
    }

    uint32_t idx = g_dlog.head;
    agent_datalog_entry_t *e = &g_dlog.ring[idx];

    e->type         = (uint32_t)type;
    e->timestamp_us = agent_time_now_us();
    e->arg1         = 0U;
    e->arg2         = 0U;
    e->payload_size = payload_size;

    if (payload_size > 0U && payload != NULL) {
        memcpy(e->payload, payload, payload_size);
    }

    g_dlog.head = (idx + 1U) % AGENT_DATALOG_RING_SIZE;
    g_dlog.count++;
    g_dlog.total_written++;
    g_dlog.last_write_us = e->timestamp_us;

    return AGENT_OK;
}

/* ---------- READ ---------- */
agent_status_t agent_datalog_read(agent_datalog_entry_t *out)
{
    if (out == NULL) {
        return AGENT_ERR;
    }

    if (g_dlog.count == 0U) {
        return AGENT_ERR;
    }

    uint32_t idx = g_dlog.tail;
    *out = g_dlog.ring[idx];

    g_dlog.tail = (idx + 1U) % AGENT_DATALOG_RING_SIZE;
    g_dlog.count--;

    return AGENT_OK;
}

/* ---------- COUNT ---------- */
uint32_t agent_datalog_count(void)
{
    return g_dlog.count;
}

/* ---------- CLEAR ---------- */
void agent_datalog_clear(void)
{
    g_dlog.head  = 0U;
    g_dlog.tail  = 0U;
    g_dlog.count = 0U;
}

/* ---------- STATE ---------- */
const agent_datalog_state_t *agent_datalog_state(void)
{
    return &g_dlog;
}