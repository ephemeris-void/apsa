#include "agent_transitions.h"
#include "agent_states.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../safety/agent_safety.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT TRANSITIONS — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Deterministic transitions only. Guarded.
 * WCET: 30 µs max
 * ============================================================ */

/* ---------- Transition Table ----------
 * [from][to] = 0 blocked / 1 allowed
 * Flash-resident const — zero RAM cost
 *
 * Recovery paths added:
 *   EMR → VIR  (AI OK → virtual sensor fallback)
 *   EMR → REC  (IMU OK → recovery)
 *   LND → VIR  (AI OK → virtual sensor fallback)
 *   LND → REC  (IMU OK → recovery)
 */
static const uint8_t transition_table[AGENT_STATE_MAX][AGENT_STATE_MAX] = {
    /*          BOOT MON DEG VIR FUS REC EMR LND */
    /* BOOT */ {  0,   1,  0,  0,  0,  0,  1,  0 },
    /* MON  */ {  0,   0,  1,  1,  0,  0,  1,  0 },
    /* DEG  */ {  0,   1,  0,  1,  1,  1,  1,  0 },
    /* VIR  */ {  0,   0,  1,  0,  1,  1,  1,  0 },
    /* FUS  */ {  0,   1,  0,  1,  0,  1,  1,  0 },
    /* REC  */ {  0,   1,  1,  1,  0,  0,  1,  0 },
    /* EMR  */ {  0,   1,  0,  1,  0,  1,  0,  1 },
    /* LND  */ {  0,   1,  1,  1,  0,  1,  1,  0 },
};

/* ---------- Allowed Check ---------- */
bool agent_transition_is_allowed(agent_state_t from, agent_state_t to)
{
    if (from >= AGENT_STATE_MAX || to >= AGENT_STATE_MAX) {
        return false;
    }
    return (transition_table[from][to] != 0U);
}

/* ---------- Main Transition (guarded) ---------- */
agent_status_t agent_transition_to(agent_state_t new_state,
                                    uint32_t     reason_code)
{
    agent_context_full_t *ctx = agent_context_get();
    agent_state_t current = ctx->current_state;

    if (new_state == current) {
        return AGENT_OK;
    }

    if (new_state >= AGENT_STATE_MAX) {
        return AGENT_ERR_INVALID_STATE;
    }

    if (!agent_transition_is_allowed(current, new_state)) {
        agent_log_event(AGENT_LOG_TRANSITION_BLOCKED,
                        ((uint32_t)current << 16) | (uint32_t)new_state,
                        reason_code);
        return AGENT_ERR_INVALID_STATE;
    }

    /* Minimum dwell time (anti-chatter) — EMERGENCY always allowed */
    if (ctx->time_in_state_us < AGENT_STATE_MIN_DWELL_US) {
        if (new_state != AGENT_STATE_EMERGENCY) {
            return AGENT_ERR_INVALID_STATE;
        }
    }

    /* Safety check before entering high-trust states */
    if (new_state == AGENT_STATE_FUSION ||
        new_state == AGENT_STATE_RECOVERY ||
        new_state == AGENT_STATE_MONITOR) {
        if (!agent_safety_check()) {
            agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                            ((uint32_t)current << 16) | (uint32_t)new_state,
                            (uint32_t)AGENT_REASON_ENVELOPE);
            return agent_transition_force(AGENT_STATE_EMERGENCY,
                                          (uint32_t)AGENT_REASON_ENVELOPE);
        }
    }

    /* Execute transition — deterministic sequence */
    agent_state_exit(current);
    agent_state_set(new_state);
    agent_state_enter(new_state);

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)current << 16) | (uint32_t)new_state,
                    reason_code);

    agent_trace_transition(current, new_state, reason_code);

    return AGENT_OK;
}

/* ---------- Force Transition (SAFETY override) ---------- */
agent_status_t agent_transition_force(agent_state_t new_state,
                                       uint32_t     reason_code)
{
    agent_context_full_t *ctx = agent_context_get();
    agent_state_t current = ctx->current_state;

    if (new_state == current) {
        return AGENT_OK;
    }

    if (new_state >= AGENT_STATE_MAX) {
        return AGENT_ERR_INVALID_STATE;
    }

    agent_state_exit(current);
    agent_state_set(new_state);
    agent_state_enter(new_state);

    agent_log_event(AGENT_LOG_TRANSITION_FORCED,
                    ((uint32_t)current << 16) | (uint32_t)new_state,
                    reason_code);

    return AGENT_OK;
}