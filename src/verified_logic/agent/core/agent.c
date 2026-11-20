#include "agent.h"
#include "agent_context.h"
#include "agent_config.h"
#include "agent_main_hook.h"
#include "../states/agent_states.h"
#include "../states/agent_transitions.h"
#include "../decisions/agent_decisions.h"
#include "../decisions/agent_priority.h"
#include "../actions/agent_actions.h"
#include "../safety/agent_safety.h"
#include "../safety/agent_watchdog.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "common/logger.h"

#include <zephyr/kernel.h>

static bool g_agent_initialized = false;
static agent_state_t g_last_logged_state = (agent_state_t)0xFF;
static agent_health_t g_last_logged_health = (agent_health_t)0xFF;

agent_status_t agent_init(void)
{
    if (g_agent_initialized) return AGENT_ERR;

    agent_context_init();
    agent_trace_init();
    agent_watchdog_init();
    agent_safety_init();

    agent_state_set(AGENT_STATE_BOOT);
    agent_state_enter(AGENT_STATE_BOOT);

    agent_log_event(AGENT_LOG_INIT, 0U, 0U);

    g_agent_initialized = true;
    LOG_I(LOG_TAG_AGENT, "[AGENT] init OK state=BOOT");
    return AGENT_OK;
}

agent_status_t agent_step(void)
{
    if (!g_agent_initialized) return AGENT_ERR_NOT_INIT;

    agent_iteration_begin();
    agent_watchdog_kick();

    agent_context_full_t *ctx = agent_context_get();
    agent_state_t cur = ctx->current_state;

    /* Read real health from IMU driver. */
    agent_health_set_imu(imu_health());

    /* In VIRTUALIZE, leave AI health alone: ai_health() reports OK
     * while IDLE / RUNNING / READY and only becomes FAIL on wrapper
     * ERROR. We must not clobber it during the AI trigger window. */
    if (cur != AGENT_STATE_VIRTUALIZE) {
        agent_health_t ai_h = (ai_health() == AGENT_OK)
                              ? AGENT_HEALTH_OK
                              : AGENT_HEALTH_FAIL;
        agent_health_set_ai(ai_h);
    }

    agent_health_refresh();

    /* DEGRADED override:
     *  - IMU fail while VIRTUALIZE/FUSION/RECOVERY -> DEGRADED
     *  - AI fail while IMU OK                      -> DEGRADED
     * FAIL (both fail) is already set by refresh(). */
    if (ctx->imu_health == AGENT_HEALTH_FAIL &&
        (ctx->current_state == AGENT_STATE_VIRTUALIZE ||
         ctx->current_state == AGENT_STATE_FUSION     ||
         ctx->current_state == AGENT_STATE_RECOVERY)) {
        ctx->system_health = AGENT_HEALTH_DEGRADED;
    }
    if (ctx->ai_health == AGENT_HEALTH_FAIL &&
        ctx->imu_health == AGENT_HEALTH_OK) {
        ctx->system_health = AGENT_HEALTH_DEGRADED;
    }

    uint32_t t1 = agent_time_now_us();

    if (!agent_safety_check()) {
        agent_safety_escalate();
        agent_iteration_end();
        return AGENT_ERR;
    }

    uint32_t t2 = agent_time_now_us();

    agent_decision_t decision;
    if (agent_decide(&decision) != AGENT_OK) {
        agent_fault_register(AGENT_FAULT_DEADLINE_MISS);
        agent_iteration_end();
        return AGENT_ERR;
    }

    uint32_t t3 = agent_time_now_us();

    if (decision.next_state != agent_get_state()) {
        agent_transition_to(decision.next_state, decision.reason_code);
    }

    if (agent_action_execute(&decision) != AGENT_OK) {
        agent_fault_register(AGENT_FAULT_DEADLINE_MISS);
        agent_iteration_end();
        return AGENT_ERR;
    }

    agent_state_run(agent_get_state());

    uint32_t t4 = agent_time_now_us();

    (void)agent_deadline_check();
    (void)agent_wcet_loop_check();

    agent_iteration_end();

    {
        agent_context_full_t *c = agent_context_get();

        if (c->current_state != g_last_logged_state) {
            g_last_logged_state = c->current_state;
            LOG_W(LOG_TAG_AGENT,
                  "[AGENT] STATE -> %s health=%d iter=%u",
                  agent_get_state_name(c->current_state),
                  (int)c->system_health,
                  (unsigned)c->iteration_count);
        }

        if (c->system_health != g_last_logged_health) {
            g_last_logged_health = c->system_health;
            LOG_W(LOG_TAG_AGENT,
                  "[AGENT] HEALTH -> %d imu=%d ai=%d",
                  (int)c->system_health,
                  (int)c->imu_health,
                  (int)c->ai_health);
        }

        uint32_t it = c->iteration_count;
        if (it > 0U && (it % 5000U) == 0U) {
            LOG_I(LOG_TAG_AGENT,
                  "[AGENT] state=%d health=%d iter=%u faults=%u wcet=%u",
                  (int)c->current_state,
                  (int)c->system_health,
                  (unsigned)it,
                  (unsigned)c->fault_count,
                  (unsigned)c->loop_wcet_us);
            LOG_I(LOG_TAG_AGENT,
                  "[TIMING] safety=%u decide=%u transition=%u state_run=%u",
                  (unsigned)(t2 - t1),
                  (unsigned)(t3 - t2),
                  (unsigned)(t3 - t2),
                  (unsigned)(t4 - t3));
        }
    }

    return AGENT_OK;
}

agent_status_t agent_shutdown(void)
{
    if (!g_agent_initialized) return AGENT_ERR_NOT_INIT;
    agent_state_enter(AGENT_STATE_LANDING);
    g_agent_initialized = false;
    return AGENT_OK;
}

agent_state_t agent_get_state(void) { return agent_context_get()->current_state; }

const char* agent_get_state_name(agent_state_t s)
{
    switch (s) {
        case AGENT_STATE_BOOT:       return "BOOT";
        case AGENT_STATE_MONITOR:    return "MONITOR";
        case AGENT_STATE_DEGRADED:   return "DEGRADED";
        case AGENT_STATE_VIRTUALIZE: return "VIRTUALIZE";
        case AGENT_STATE_FUSION:     return "FUSION";
        case AGENT_STATE_RECOVERY:   return "RECOVERY";
        case AGENT_STATE_EMERGENCY:  return "EMERGENCY";
        case AGENT_STATE_LANDING:    return "LANDING";
        default:                     return "UNKNOWN";
    }
}

agent_health_t agent_get_health(void) { return agent_context_get()->system_health; }
uint32_t agent_get_iteration_count(void) { return agent_context_get()->iteration_count; }
uint32_t agent_get_fault_count(void) { return agent_context_get()->fault_count; }