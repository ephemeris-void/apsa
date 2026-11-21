#include "agent_datalog_ring.h"
#include "agent_datalog.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"

/* ============================================================
 * AGENT DATALOG RING — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 1 µs per call
 * ============================================================ */

static bool g_ring_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_datalog_ring_init(void)
{
    g_ring_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_datalog_ring_reset(void)
{
    return agent_datalog_ring_init();
}

/* ---------- CAPACITY ---------- */
uint32_t agent_datalog_ring_capacity(void)
{
    return AGENT_DATALOG_RING_SIZE;
}

/* ---------- USAGE ---------- */
uint32_t agent_datalog_ring_usage(void)
{
    if (!g_ring_initialized) {
        return 0U;
    }

    const agent_datalog_state_t *s = agent_datalog_state();
    if (s == NULL) {
        return 0U;
    }
    return s->count;
}

/* ---------- IS FULL ---------- */
bool agent_datalog_ring_is_full(void)
{
    if (!g_ring_initialized) {
        return true;
    }

    const agent_datalog_state_t *s = agent_datalog_state();
    if (s == NULL) {
        return true;
    }
    return (s->count >= AGENT_DATALOG_RING_SIZE);
}

/* ---------- IS EMPTY ---------- */
bool agent_datalog_ring_is_empty(void)
{
    if (!g_ring_initialized) {
        return true;
    }

    const agent_datalog_state_t *s = agent_datalog_state();
    if (s == NULL) {
        return true;
    }
    return (s->count == 0U);
}