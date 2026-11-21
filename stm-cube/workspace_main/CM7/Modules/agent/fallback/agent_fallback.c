#include "agent_fallback.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT FALLBACK — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 200 µs per execute
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_fallback_entry_t g_entries[AGENT_FALLBACK_MAX_LEVELS];
static AGENT_DTCM agent_fallback_state_t g_state;
static AGENT_DTCM agent_fallback_stats_t g_stats;
static bool                              g_fallback_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_fallback_init(void)
{
    memset(g_entries, 0, sizeof(g_entries));    /* zero-init incl. reserved[] */
    memset(&g_state, 0, sizeof(g_state));       /* zero-init incl. reserved[] */
    memset(&g_stats, 0, sizeof(g_stats));       /* zero-init incl. reserved[] */

    for (uint32_t i = 0U; i < AGENT_FALLBACK_MAX_LEVELS; i++) {
        g_entries[i].level          = (agent_fallback_level_t)i;
        g_entries[i].name           = "unset";
        g_entries[i].fn             = NULL;
        g_entries[i].wcet_budget_us = 0U;
        g_entries[i].enabled        = false;
    }

    g_state.active_level    = AGENT_FALLBACK_LEVEL_PRIMARY;
    g_state.requested_level = AGENT_FALLBACK_LEVEL_PRIMARY;
    g_state.switch_count    = 0U;
    g_state.fail_count      = 0U;
    g_state.last_switch_us  = agent_time_now_us();

    g_fallback_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_fallback_reset(void)
{
    g_fallback_initialized = false;
    return agent_fallback_init();
}

/* ---------- REGISTER ---------- */
agent_status_t agent_fallback_register(agent_fallback_level_t level,
                                       const char             *name,
                                       agent_fallback_fn_t     fn,
                                       uint32_t                wcet_us)
{
    if (!g_fallback_initialized)                             return AGENT_ERR_NOT_INIT;
    if (level >= AGENT_FALLBACK_MAX_LEVELS || fn == NULL)    return AGENT_ERR;

    g_entries[level].level          = level;
    g_entries[level].name           = (name != NULL) ? name : "unnamed";
    g_entries[level].fn             = fn;
    g_entries[level].wcet_budget_us = wcet_us;
    g_entries[level].enabled        = true;

    return AGENT_OK;
}

/* ---------- REQUEST ---------- */
agent_status_t agent_fallback_request(agent_fallback_level_t level)
{
    if (!g_fallback_initialized)             return AGENT_ERR_NOT_INIT;
    if (level >= AGENT_FALLBACK_MAX_LEVELS)  return AGENT_ERR;

    g_state.requested_level = level;
    return AGENT_OK;
}

/* ---------- SWITCH ---------- */
agent_status_t agent_fallback_switch(agent_fallback_level_t level)
{
    if (!g_fallback_initialized)             return AGENT_ERR_NOT_INIT;
    if (level >= AGENT_FALLBACK_MAX_LEVELS)  return AGENT_ERR;

    if (level == g_state.active_level) {
        return AGENT_OK;
    }

    agent_fallback_level_t prev = g_state.active_level;

    g_state.active_level   = level;
    g_state.switch_count++;
    g_state.last_switch_us = agent_time_now_us();

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)prev << 16) | (uint32_t)level,
                    0x01U);

    return AGENT_OK;
}

/* ---------- EXECUTE ----------
 * Try levels starting from requested, wrapping around.
 * First success wins. All fails → AGENT_ERR_HW.
 */
agent_status_t agent_fallback_execute(agent_sensor_sample_t *out,
                                      agent_fallback_level_t *used_level)
{
    if (out == NULL)             return AGENT_ERR;
    if (!g_fallback_initialized) return AGENT_ERR_NOT_INIT;

    agent_fallback_level_t start = g_state.requested_level;

    for (uint32_t i = 0U; i < AGENT_FALLBACK_MAX_LEVELS; i++) {

        agent_fallback_level_t lvl =
            (agent_fallback_level_t)((start + i) % AGENT_FALLBACK_MAX_LEVELS);

        if (!g_entries[lvl].enabled || g_entries[lvl].fn == NULL) {
            continue;
        }

        uint32_t t0 = agent_time_now_us();
        agent_status_t st = g_entries[lvl].fn(out);
        uint32_t dt = agent_time_now_us() - t0;

        g_stats.total_calls++;
        g_stats.level_calls[lvl]++;

        /* WCET enforcement */
        if (g_entries[lvl].wcet_budget_us != 0U &&
            dt > g_entries[lvl].wcet_budget_us) {
            agent_log_event(AGENT_LOG_WCET_VIOLATE, (uint32_t)lvl, dt);
        }

        if (st == AGENT_OK) {
            g_stats.success_calls++;
            g_state.active_level = lvl;

            if (lvl != start) {
                g_state.switch_count++;
                g_state.last_switch_us = agent_time_now_us();

                agent_log_event(AGENT_LOG_TRANSITION,
                                ((uint32_t)start << 16) | (uint32_t)lvl,
                                0U);
            }

            if (used_level != NULL) {
                *used_level = lvl;
            }
            return AGENT_OK;
        }

        g_stats.failed_calls++;
        g_state.fail_count++;
    }

    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0xFFFFU, g_stats.failed_calls);
    return AGENT_ERR_HW;
}

/* ---------- QUERIES ---------- */
agent_fallback_level_t agent_fallback_active(void)
{
    return g_state.active_level;
}

const agent_fallback_state_t *agent_fallback_state(void)
{
    return &g_state;
}

const agent_fallback_stats_t *agent_fallback_stats(void)
{
    return &g_stats;
}