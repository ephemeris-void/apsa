#include "agent_escalation.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT ESCALATION — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per update
 * ============================================================ */

/* PERF: state + log in DTCM (zero wait state) */
static AGENT_DTCM agent_escalation_state_t g_state;
static AGENT_DTCM agent_escalation_log_t   g_log;

/* ---------- Log event ---------- */
static void log_event(agent_escalation_level_t level,
                      agent_escalation_trigger_t trigger,
                      uint32_t reason_code)
{
    uint32_t idx = g_log.head;

    g_log.history[idx].level        = level;
    g_log.history[idx].trigger      = trigger;
    g_log.history[idx].reason_code  = reason_code;
    g_log.history[idx].timestamp_us = agent_time_now_us();
    g_log.history[idx].fault_count  = agent_fault_get_count();

    g_log.head = (idx + 1U) % AGENT_ESCALATION_HISTORY_DEPTH;
    if (g_log.count < AGENT_ESCALATION_HISTORY_DEPTH) {
        g_log.count++;
    }
}

/* ---------- INIT ---------- */
agent_status_t agent_escalation_init(void)
{
    memset(&g_state, 0, sizeof(g_state));    /* zero-init incl. reserved[] */
    memset(&g_log, 0, sizeof(g_log));        /* zero-init */

    g_state.current_level      = AGENT_ESCALATION_LEVEL_NONE;
    g_state.max_level          = AGENT_ESCALATION_LEVEL_NONE;
    g_state.escalation_count   = 0U;
    g_state.deescalation_count = 0U;
    g_state.last_change_us     = agent_time_now_us();
    g_state.stable_dwell_us    = AGENT_STATE_MIN_DWELL_US;

    return AGENT_OK;
}

agent_status_t agent_escalation_reset(void)
{
    return agent_escalation_init();
}

/* ---------- RAISE ---------- */
agent_status_t agent_escalation_raise(agent_escalation_trigger_t trigger,
                                      uint32_t                   reason_code)
{
    if (g_state.current_level >= AGENT_ESCALATION_LEVEL_CRITICAL) {
        return AGENT_OK;
    }

    agent_escalation_level_t next =
        (agent_escalation_level_t)(g_state.current_level + 1U);

    g_state.current_level = next;
    if (next > g_state.max_level) {
        g_state.max_level = next;
    }
    g_state.escalation_count++;
    g_state.last_change_us = agent_time_now_us();

    log_event(next, trigger, reason_code);

    agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                    ((uint32_t)next << 16) | (uint32_t)trigger,
                    reason_code);

    return AGENT_OK;
}

/* ---------- LOWER ---------- */
agent_status_t agent_escalation_lower(agent_escalation_trigger_t trigger)
{
    if (g_state.current_level <= AGENT_ESCALATION_LEVEL_NONE) {
        return AGENT_OK;
    }

    uint32_t now = agent_time_now_us();

    /* Enforce minimum dwell before de-escalation */
    if ((now - g_state.last_change_us) < g_state.stable_dwell_us) {
        return AGENT_ERR_INVALID_STATE;
    }

    agent_escalation_level_t next =
        (agent_escalation_level_t)(g_state.current_level - 1U);

    g_state.current_level = next;
    g_state.deescalation_count++;
    g_state.last_change_us = now;

    log_event(next, trigger, 0U);

    return AGENT_OK;
}

/* ---------- AUTO UPDATE ---------- */
void agent_escalation_auto_update(void)
{
    agent_context_full_t *ctx = agent_context_get();

    /* 1. System failure → immediate CRITICAL */
    if (ctx->system_health == AGENT_HEALTH_FAIL) {
        if (g_state.current_level < AGENT_ESCALATION_LEVEL_CRITICAL) {
            (void)agent_escalation_raise(AGENT_ESCALATION_TRIGGER_FAULT,
                                         (uint32_t)AGENT_FAULT_IMU_FAIL);
        }
        return;
    }

    /* 2. WCET violations */
    if (ctx->wcet_violate_count > AGENT_FAULT_ESCALATE_THRESHOLD) {
        (void)agent_escalation_raise(AGENT_ESCALATION_TRIGGER_WCET,
                                     ctx->wcet_violate_count);
    }

    /* 3. Deadline misses */
    if (ctx->deadline_miss_count > AGENT_FAULT_ESCALATE_THRESHOLD) {
        (void)agent_escalation_raise(AGENT_ESCALATION_TRIGGER_TIMEOUT,
                                     ctx->deadline_miss_count);
    }

    /* 4. Low confidence */
    if (ctx->confidence < AGENT_CONFIDENCE_MIN) {
        (void)agent_escalation_raise(AGENT_ESCALATION_TRIGGER_CONFIDENCE,
                                     (uint32_t)(ctx->confidence * 1000.0f));
    }

    /* 5. De-escalation when stable */
    if (ctx->system_health == AGENT_HEALTH_OK &&
        ctx->confidence >= AGENT_CONFIDENCE_RECOVER) {

        uint32_t now = agent_time_now_us();

        if ((now - g_state.last_change_us) > (g_state.stable_dwell_us * 10U)) {
            (void)agent_escalation_lower(AGENT_ESCALATION_TRIGGER_NONE);
        }
    }
}

/* ---------- QUERIES ---------- */
agent_escalation_level_t agent_escalation_current(void)
{
    return g_state.current_level;
}

agent_escalation_level_t agent_escalation_max(void)
{
    return g_state.max_level;
}

const agent_escalation_state_t *agent_escalation_state(void)
{
    return &g_state;
}

const agent_escalation_log_t *agent_escalation_log(void)
{
    return &g_log;
}