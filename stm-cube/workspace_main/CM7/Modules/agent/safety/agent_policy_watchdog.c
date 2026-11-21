#include "agent_policy_watchdog.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY WATCHDOG — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per kick
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_wdt_state_t g_wdt;
static bool                                g_wdt_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_watchdog_init(void)
{
    memset(&g_wdt, 0, sizeof(g_wdt));

    for (uint32_t i = 0U; i < AGENT_POLICY_ID_COUNT; i++) {
        g_wdt.entries[i].policy_id        = (agent_policy_id_t)i;
        g_wdt.entries[i].last_call_us     = 0U;
        g_wdt.entries[i].last_latency_us  = 0U;
        g_wdt.entries[i].worst_latency_us = 0U;
        g_wdt.entries[i].call_count       = 0U;
        g_wdt.entries[i].timeout_count    = 0U;
        g_wdt.entries[i].fail_count       = 0U;
        g_wdt.entries[i].result           = AGENT_POLICY_WDT_OK;
    }

    g_wdt.total_calls     = 0U;
    g_wdt.total_timeouts  = 0U;
    g_wdt.total_fails     = 0U;
    g_wdt.last_update_us  = agent_time_now_us();

    g_wdt_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_watchdog_reset(void)
{
    g_wdt_initialized = false;
    return agent_policy_watchdog_init();
}

/* ---------- KICK ---------- */
agent_status_t agent_policy_watchdog_kick(agent_policy_id_t id,
                                          uint32_t          latency_us,
                                          agent_status_t    status)
{
    if (!g_wdt_initialized)             return AGENT_ERR_NOT_INIT;
    if (id >= AGENT_POLICY_ID_COUNT)    return AGENT_ERR;

    agent_policy_wdt_entry_t *e = &g_wdt.entries[id];
    uint32_t now = agent_time_now_us();

    e->last_call_us    = now;
    e->last_latency_us = latency_us;
    e->call_count++;

    if (latency_us > e->worst_latency_us) {
        e->worst_latency_us = latency_us;
    }

    g_wdt.total_calls++;

    /* Failure path */
    if (status != AGENT_OK) {
        e->fail_count++;
        g_wdt.total_fails++;

        if (e->fail_count >= AGENT_POLICY_WDT_MAX_FAILS) {
            e->result = AGENT_POLICY_WDT_FAILED;
            agent_log_event(AGENT_LOG_AI_FAIL, (uint32_t)id, e->fail_count);
        } else {
            e->result = AGENT_POLICY_WDT_WARN;
        }
        return AGENT_ERR;
    }

    /* Latency violation */
    if (latency_us > AGENT_POLICY_WDT_MAX_LATENCY_US) {
        e->timeout_count++;
        g_wdt.total_timeouts++;
        e->result = AGENT_POLICY_WDT_TIMEOUT;
        agent_log_event(AGENT_LOG_WCET_VIOLATE, (uint32_t)id, latency_us);
        return AGENT_ERR_TIMEOUT;
    }

    e->result = AGENT_POLICY_WDT_OK;
    return AGENT_OK;
}

/* ---------- UPDATE (timeout sweep) ---------- */
void agent_policy_watchdog_update(void)
{
    if (!g_wdt_initialized) {
        return;
    }

    uint32_t now = agent_time_now_us();

    for (uint32_t i = 0U; i < AGENT_POLICY_ID_COUNT; i++) {

        agent_policy_wdt_entry_t *e = &g_wdt.entries[i];

        /* Only check policies that have been used */
        if (e->call_count == 0U) {
            continue;
        }

        uint32_t dt = now - e->last_call_us;

        if (dt > AGENT_POLICY_WDT_TIMEOUT_US &&
            e->result != AGENT_POLICY_WDT_FAILED) {

            e->result = AGENT_POLICY_WDT_TIMEOUT;
            e->timeout_count++;
            g_wdt.total_timeouts++;

            agent_log_event(AGENT_LOG_WCET_VIOLATE,
                            ((uint32_t)i << 16) | 0x01U, dt);
        }
    }

    g_wdt.last_update_us = now;
}

/* ---------- IS HEALTHY ---------- */
bool agent_policy_watchdog_is_healthy(agent_policy_id_t id)
{
    if (id >= AGENT_POLICY_ID_COUNT) {
        return false;
    }
    return (g_wdt.entries[id].result == AGENT_POLICY_WDT_OK);
}

/* ---------- QUERIES ---------- */
const agent_policy_wdt_entry_t *agent_policy_watchdog_entry(agent_policy_id_t id)
{
    if (id >= AGENT_POLICY_ID_COUNT) {
        return NULL;
    }
    return &g_wdt.entries[id];
}

const agent_policy_wdt_state_t *agent_policy_watchdog_state(void)
{
    return &g_wdt;
}