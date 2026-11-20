#include "agent_decisions.h"
#include "agent_priority.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../states/agent_states.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../actions/agent_actions.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

#include <zephyr/kernel.h>

typedef struct {
    agent_health_t   imu_health;
    agent_health_t   ai_health;
    uint32_t         time_in_state_us;
    uint32_t         fail_count;
    uint32_t         ok_count;
    uint32_t         imu_fail_accum_us;
    bool             safety_ok;
} perception_t;

static perception_t g_perception;

void agent_decide_perceive(void)
{
    agent_context_full_t *ctx = agent_context_get();
    g_perception.imu_health        = ctx->imu_health;
    g_perception.ai_health         = ctx->ai_health;
    g_perception.time_in_state_us  = agent_time_in_state_us();
    g_perception.fail_count        = ctx->imu_consecutive_fail;
    g_perception.ok_count          = ctx->imu_consecutive_ok;
    g_perception.imu_fail_accum_us = ctx->imu_fail_accum_us;
    g_perception.safety_ok         = agent_safety_check();
}

agent_state_t agent_decide_next_state(void)
{
    agent_state_t current = agent_context_get()->current_state;

    if (current != AGENT_STATE_VIRTUALIZE &&
        current != AGENT_STATE_FUSION &&
        current != AGENT_STATE_RECOVERY &&
        current != AGENT_STATE_EMERGENCY) {
        if (!g_perception.safety_ok) return AGENT_STATE_EMERGENCY;
    }

    switch (current) {

    case AGENT_STATE_BOOT:
        return AGENT_STATE_MONITOR;

    case AGENT_STATE_MONITOR:
        if (g_perception.imu_health == AGENT_HEALTH_FAIL) {
            if (g_perception.ai_health == AGENT_HEALTH_FAIL) {
                return AGENT_STATE_EMERGENCY;
            }
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
        if (g_perception.imu_health == AGENT_HEALTH_FAIL) {
            if (g_perception.ai_health == AGENT_HEALTH_FAIL) {
                return AGENT_STATE_EMERGENCY;
            }
            if (g_perception.fail_count >= AGENT_FAIL_CONFIRM_COUNT) {
                return AGENT_STATE_VIRTUALIZE;
            }
        }
        return AGENT_STATE_DEGRADED;

    case AGENT_STATE_VIRTUALIZE: {
        ai_status_t st = ai_status();
        if (ai_output_ready()) return AGENT_STATE_FUSION;
        if (st == AI_STATUS_ERROR || g_perception.ai_health == AGENT_HEALTH_FAIL) {
            return AGENT_STATE_EMERGENCY;
        }
        return AGENT_STATE_VIRTUALIZE;
    }

    case AGENT_STATE_FUSION:
        if (g_perception.ai_health == AGENT_HEALTH_FAIL) {
            if (g_perception.imu_health == AGENT_HEALTH_OK) {
                return AGENT_STATE_MONITOR;
            }
            return AGENT_STATE_EMERGENCY;
        }
        if (g_perception.imu_health == AGENT_HEALTH_OK &&
            g_perception.ok_count >= AGENT_RECOVER_CONFIRM_COUNT) {
            return AGENT_STATE_RECOVERY;
        }
        if (g_perception.imu_health == AGENT_HEALTH_FAIL &&
            g_perception.imu_fail_accum_us > AGENT_VIRTUAL_ONLY_TIMEOUT_US) {
            return AGENT_STATE_LANDING;
        }
        return AGENT_STATE_FUSION;

    case AGENT_STATE_RECOVERY:
        if (g_perception.imu_health == AGENT_HEALTH_OK) {
            return AGENT_STATE_MONITOR;
        }
        if (g_perception.imu_health == AGENT_HEALTH_FAIL) {
            if (g_perception.ai_health == AGENT_HEALTH_FAIL) {
                return AGENT_STATE_EMERGENCY;
            }
            return AGENT_STATE_VIRTUALIZE;
        }
        return AGENT_STATE_RECOVERY;

    case AGENT_STATE_EMERGENCY:
        if (agent_safety_can_land_standalone() ||
            agent_safety_can_land() ||
            g_perception.time_in_state_us > AGENT_EMERGENCY_LAND_TIMEOUT_US) {
            return AGENT_STATE_LANDING;
        }
        return AGENT_STATE_EMERGENCY;

    case AGENT_STATE_LANDING:
        if (g_perception.imu_health == AGENT_HEALTH_OK) {
            return AGENT_STATE_MONITOR;
        }
        return AGENT_STATE_LANDING;

    default:
        return AGENT_STATE_EMERGENCY;
    }
}

uint32_t agent_decide_reason(void)
{
    agent_state_t current = agent_context_get()->current_state;

    if (!g_perception.safety_ok && current != AGENT_STATE_EMERGENCY) {
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
        if (ai_status() == AI_STATUS_ERROR || g_perception.ai_health == AGENT_HEALTH_FAIL) {
            return (uint32_t)AGENT_REASON_CONFIDENCE;
        }
        break;

    case AGENT_STATE_FUSION:
        if (g_perception.ai_health == AGENT_HEALTH_FAIL) {
            return (uint32_t)AGENT_REASON_CONFIDENCE;
        }
        if (g_perception.imu_health == AGENT_HEALTH_FAIL &&
            g_perception.imu_fail_accum_us > AGENT_VIRTUAL_ONLY_TIMEOUT_US) {
            return (uint32_t)AGENT_REASON_TIMEOUT;
        }
        break;

    default:
        break;
    }

    return (uint32_t)AGENT_REASON_NONE;
}

agent_status_t agent_decide(agent_decision_t *out)
{
    if (out == NULL) return AGENT_ERR;

    agent_trace_decision_begin();
    agent_decide_perceive();

    out->next_state   = agent_decide_next_state();
    out->priority     = agent_decide_priority();
    out->action_id    = (uint8_t)AGENT_ACTION_NONE;
    out->reason_code  = agent_decide_reason();
    out->timestamp_us = agent_time_now_us();
    out->reserved[0]  = 0U;
    out->reserved[1]  = 0U;

    agent_trace_decision_end();
    return AGENT_OK;
}