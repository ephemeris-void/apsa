#include "agent_actions.h"
#include "agent_actions_ai.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_control_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_imu_if.h"
#include "../safety/agent_safety.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include "../ipc/agent_ipc.h"

/* ============================================================
 * AGENT ACTIONS — IMPLEMENTATION
 * ============================================================ */

agent_status_t agent_action_execute(const agent_decision_t *decision)
{
    if (decision == NULL) {
        return AGENT_ERR;
    }

    agent_trace_action_begin(decision->action_id);

    agent_status_t st = AGENT_OK;

    switch (decision->action_id) {

    case AGENT_ACTION_NONE:
        break;

    case AGENT_ACTION_SET_MODE:
        st = agent_action_set_mode((agent_control_mode_t)decision->reason_code);
        break;

    case AGENT_ACTION_AI_TRIGGER:
        st = agent_action_ai_trigger();
        break;

    case AGENT_ACTION_AI_ABORT:
        st = agent_action_ai_abort();
        break;

    case AGENT_ACTION_EKF_RESET:
        st = agent_action_ekf_reset();
        break;

    case AGENT_ACTION_EMERGENCY:
        st = agent_action_emergency();
        break;

    case AGENT_ACTION_EMERGENCY_HOLD:
        st = agent_action_emergency_hold();
        break;

    case AGENT_ACTION_LANDING_STEP:
        st = agent_action_landing_step();
        break;

    case AGENT_ACTION_IPC_SEND:
        st = agent_action_ipc_send(decision->reason_code,
                                   decision->timestamp_us);
        break;

    case AGENT_ACTION_GROUND_ALERT:
        st = agent_action_ground_alert(decision->reason_code);
        break;

    case AGENT_ACTION_LOG_EVENT:
        st = agent_action_log_event(decision->reason_code, 0U, 0U);
        break;

    case AGENT_ACTION_PERSIST_STATE:
        st = agent_action_persist_state();
        break;

    default:
        st = AGENT_ERR;
        break;
    }

    agent_trace_action_end(decision->action_id);

    return st;
}

agent_status_t agent_action_set_mode(agent_control_mode_t mode)
{
    return control_set_mode(mode);
}

agent_status_t agent_action_ai_trigger(void)
{
    if (ai_status() == AI_STATUS_RUNNING) {
        return AGENT_OK;
    }

    agent_log_event(AGENT_LOG_AI_TRIGGER, 0U, 0U);
    return ai_trigger();
}

agent_status_t agent_action_ai_abort(void)
{
    agent_log_event(AGENT_LOG_AI_ABORT, 0U, 0U);
    return agent_actions_ai_abort();
}

agent_status_t agent_action_ekf_reset(void)
{
    agent_log_event(AGENT_LOG_EKF_RESET, 0U, 0U);
    return ekf_init();
}

agent_status_t agent_action_emergency(void)
{
    agent_log_event(AGENT_LOG_EMERGENCY, 0U, 0U);
    agent_action_ground_alert(0xEE01U);
    agent_action_ipc_send(AGENT_IPC_MSG_EMERGENCY, 0U);
    return AGENT_OK;
}

agent_status_t agent_action_emergency_hold(void)
{
    control_set_mode(AGENT_MODE_EMERGENCY);

    static uint32_t last_alert_us = 0U;
    uint32_t now = agent_time_now_us();

    if ((now - last_alert_us) > AGENT_GROUND_ALERT_TIMEOUT_US) {
        last_alert_us = now;
        agent_action_ground_alert(0xEE02U);
    }

    return AGENT_OK;
}

agent_status_t agent_action_landing_step(void)
{
    control_set_mode(AGENT_MODE_LANDING);
    return AGENT_OK;
}

agent_status_t agent_action_ipc_send(uint32_t msg_id, uint32_t payload)
{
    return agent_ipc_send(msg_id, payload, 0U);
}

agent_status_t agent_action_ground_alert(uint32_t code)
{
    return agent_ipc_send(AGENT_IPC_MSG_GROUND_ALERT, code, 0U);
}

agent_status_t agent_action_log_event(uint32_t code,
                                       uint32_t arg1,
                                       uint32_t arg2)
{
    agent_log_event(code, arg1, arg2);
    return AGENT_OK;
}

agent_status_t agent_action_persist_state(void)
{
    agent_log_event(AGENT_LOG_PERSIST, 0U, 0U);
    return AGENT_OK;
}
