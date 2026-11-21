#include "agent_decisions.h"
#include "agent_priority.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../states/agent_states.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT DECISIONS — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 200 µs
 * Deterministic: same input → same output
 * ============================================================ */

/* ---------- Perception Snapshot ---------- */
typedef struct {
    agent_health_t   imu_health;
    agent_health_t   ai_health;
    agent_health_t   ekf_health;
    agent_health_t   baro_health;
    agent_health_t   gps_health;
    float            confidence;
    uint32_t         time_in_state_us;
    uint32_t         fail_count;
    uint32_t         ok_count;
    bool             safety_ok;
} perception_t;

static perception_t g_perception;

/* ---------- PERCEPTION ---------- */
void agent_decide_perceive(void)
{
    agent_context_full_t *ctx = agent_context_get();

    g_perception.imu_health       = ctx->imu_health;
    g_perception.ai_health        = ctx->ai_health;
    g_perception.ekf_health       = AGENT_HEALTH_OK;
    g_perception.baro_health      = ctx->baro_health;
    g_perception.gps_health       = ctx->gps_health;
    g_perception.confidence       = ctx->confidence;
    g_perception.time_in_state_us = ctx->time_in_state_us;
    g_perception.fail_count       = ctx->imu_consecutive_fail;
    g_perception.ok_count         = ctx->imu_consecutive_ok;
    g_perception.safety_ok        = agent_safety_check();
}

/* ---------- REASONING ---------- */
agent_state_t agent_decide_next_state(void)
{
    agent_state_t current = agent_context_get()->current_state;

    /* EMERGENCY always wins — but only if not in EMERGENCY/LANDING
     * recovery window. This allows exit from EMERGENCY/LANDING. */
    if (!g_perception.safety_ok &&
        current != AGENT_STATE_EMERGENCY &&
        current != AGENT_STATE_LANDING) {
        return AGENT_STATE_EMERGENCY;
    }

    switch (current) {

    case AGENT_STATE_BOOT:
        return AGENT_STATE_MONITOR;

    case AGENT_STATE_MONITOR:
        if (g_perception.imu_health == AGENT_HEALTH_FAIL) {
            return AGENT_STATE_VIRTUALIZE;
        }
        if (g_perception.imu_health == AGENT_HEALTH_DEGRADED) {
            return AGENT_STATE_DEGRADED;
        }
        return AGENT_STATE_MONITOR;

    case AGENT_STATE_DEGRADED:
        if (g_perception.imu_health == AGENT_HEALTH_OK &&
            g_perception.ok_count >= AGENT_RECOVER_CONFIRM_COUNT) {
            return AGENT_STATE_RECOVERY;
        }
        if (g_perception.imu_health == AGENT_HEALTH_FAIL &&
            g_perception.fail_count >= AGENT_FAIL_CONFIRM_COUNT) {
            return AGENT_STATE_VIRTUALIZE;
        }
        return AGENT_STATE_DEGRADED;

    case AGENT_STATE_VIRTUALIZE:
        if (g_perception.ai_health == AGENT_HEALTH_OK &&
            g_perception.confidence >= AGENT_CONFIDENCE_RECOVER) {
            return AGENT_STATE_FUSION;
        }
        if (g_perception.ai_health == AGENT_HEALTH_FAIL) {
            return AGENT_STATE_EMERGENCY;
        }
        if (g_perception.time_in_state_us > AGENT_STATE_TIMEOUT_US) {
            return AGENT_STATE_EMERGENCY;
        }
        return AGENT_STATE_VIRTUALIZE;

    case AGENT_STATE_FUSION:
        /* IMU recovered → RECOVERY */
        if (g_perception.imu_health == AGENT_HEALTH_OK &&
            g_perception.ok_count >= AGENT_RECOVER_CONFIRM_COUNT) {
            return AGENT_STATE_RECOVERY;
        }
        /* IMU failed again → VIRTUALIZE */
        if (g_perception.imu_health == AGENT_HEALTH_FAIL &&
            g_perception.fail_count >= AGENT_FAIL_CONFIRM_COUNT) {
            return AGENT_STATE_VIRTUALIZE;
        }
        /* Both real and virtual bad → EMERGENCY */
        if (g_perception.confidence < AGENT_CONFIDENCE_MIN) {
            return AGENT_STATE_EMERGENCY;
        }
        return AGENT_STATE_FUSION;

    case AGENT_STATE_RECOVERY:
        if (g_perception.imu_health == AGENT_HEALTH_OK) {
            return AGENT_STATE_MONITOR;
        }
        if (g_perception.imu_health == AGENT_HEALTH_FAIL) {
            return AGENT_STATE_VIRTUALIZE;
        }
        return AGENT_STATE_RECOVERY;

    case AGENT_STATE_EMERGENCY:
        /* 1. Real-time recovery — IMU OK → RECOVERY (highest priority) */
        if (g_perception.imu_health == AGENT_HEALTH_OK &&
            g_perception.ok_count >= AGENT_RECOVER_CONFIRM_COUNT) {
            return AGENT_STATE_RECOVERY;
        }
        /* 2. Can land safely → LANDING */
        if (agent_safety_can_land()) {
            return AGENT_STATE_LANDING;
        }
        /* 3. Stay in EMERGENCY — no AI-based soft recovery */
        return AGENT_STATE_EMERGENCY;

    case AGENT_STATE_LANDING:
        /* SAFETY: Minimum dwell in LANDING — avoid instant bounce */
        if (g_perception.time_in_state_us < AGENT_LANDING_MIN_DWELL_US) {
            return AGENT_STATE_LANDING;
        }

        /* 1. IMU recovered → RECOVERY */
        if (g_perception.imu_health == AGENT_HEALTH_OK &&
            g_perception.ok_count >= AGENT_RECOVER_CONFIRM_COUNT) {
            return AGENT_STATE_RECOVERY;
        }

        /* 2. AI recovered → VIRTUALIZE (fallback path) */
        if (g_perception.ai_health == AGENT_HEALTH_OK) {
            return AGENT_STATE_VIRTUALIZE;
        }

        /* 3. IMU degraded (soft fail) → DEGRADED */
        if (g_perception.imu_health == AGENT_HEALTH_DEGRADED) {
            return AGENT_STATE_DEGRADED;
        }

        /* 4. Stay in LANDING (worst case) */
        return AGENT_STATE_LANDING;

    default:
        return AGENT_STATE_EMERGENCY;
    }
}

/* ---------- REASON CODES ---------- */
uint32_t agent_decide_reason(void)
{
    agent_state_t current = agent_context_get()->current_state;

    if (!g_perception.safety_ok &&
        current != AGENT_STATE_EMERGENCY &&
        current != AGENT_STATE_LANDING) {
        return (uint32_t)AGENT_REASON_ENVELOPE;
    }

    switch (current) {
    case AGENT_STATE_MONITOR:
        if (g_perception.imu_health == AGENT_HEALTH_FAIL)     return (uint32_t)AGENT_REASON_TIMEOUT;
        if (g_perception.imu_health == AGENT_HEALTH_DEGRADED) return (uint32_t)AGENT_REASON_DRIFT;
        break;

    case AGENT_STATE_DEGRADED:
        if (g_perception.ok_count >= AGENT_RECOVER_CONFIRM_COUNT) return (uint32_t)AGENT_REASON_NONE;
        if (g_perception.fail_count >= AGENT_FAIL_CONFIRM_COUNT)  return (uint32_t)AGENT_REASON_INNOVATION;
        break;

    case AGENT_STATE_VIRTUALIZE:
        if (g_perception.ai_health == AGENT_HEALTH_FAIL)            return (uint32_t)AGENT_REASON_CONFIDENCE;
        if (g_perception.time_in_state_us > AGENT_STATE_TIMEOUT_US) return (uint32_t)AGENT_REASON_TIMEOUT;
        break;

    case AGENT_STATE_FUSION:
        if (g_perception.confidence < AGENT_CONFIDENCE_MIN) return (uint32_t)AGENT_REASON_CONFIDENCE;
        break;

    case AGENT_STATE_EMERGENCY:
        if (g_perception.imu_health == AGENT_HEALTH_OK)     return (uint32_t)AGENT_REASON_NONE;
        if (g_perception.ai_health == AGENT_HEALTH_OK)      return (uint32_t)AGENT_REASON_CONFIDENCE;
        break;

    case AGENT_STATE_LANDING:
        if (g_perception.imu_health == AGENT_HEALTH_OK)       return (uint32_t)AGENT_REASON_NONE;
        if (g_perception.ai_health == AGENT_HEALTH_OK)        return (uint32_t)AGENT_REASON_CONFIDENCE;
        if (g_perception.imu_health == AGENT_HEALTH_DEGRADED) return (uint32_t)AGENT_REASON_DRIFT;
        break;

    default:
        break;
    }

    return (uint32_t)AGENT_REASON_NONE;
}

/* ---------- MAIN DECIDE ---------- */
agent_status_t agent_decide(agent_decision_t *out)
{
    if (out == NULL) {
        return AGENT_ERR;
    }

    agent_trace_decision_begin();

    agent_decide_perceive();

    out->next_state   = agent_decide_next_state();
    out->reason_code  = agent_decide_reason();
    out->priority     = agent_decide_priority();
    out->action_id    = 0U;
    out->timestamp_us = agent_time_now_us();

    agent_trace_decision_end();

    return AGENT_OK;
}

/* ---------- DECIDE PRIORITY ---------- */
agent_priority_t agent_decide_priority(void)
{
    return agent_priority_evaluate();
}