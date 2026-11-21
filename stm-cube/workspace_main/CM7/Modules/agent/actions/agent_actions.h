#ifndef AGENT_ACTIONS_H
#define AGENT_ACTIONS_H

#include "../core/agent_types.h"
#include "../interfaces/agent_control_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT ACTIONS — FROZEN
 * ============================================================
 * Executes decisions. Every action is deterministic and bounded.
 * Actions trigger external modules via interfaces only.
 * WCET: 100 µs max per action.
 * ============================================================ */

/* ---------- Action IDs ---------- */
typedef enum {
    AGENT_ACTION_NONE           = 0x00,
    AGENT_ACTION_SET_MODE       = 0x01,
    AGENT_ACTION_AI_TRIGGER     = 0x02,
    AGENT_ACTION_AI_ABORT       = 0x03,
    AGENT_ACTION_EKF_RESET      = 0x04,
    AGENT_ACTION_EMERGENCY      = 0x05,
    AGENT_ACTION_EMERGENCY_HOLD = 0x06,
    AGENT_ACTION_LANDING_STEP   = 0x07,
    AGENT_ACTION_IPC_SEND       = 0x08,
    AGENT_ACTION_GROUND_ALERT   = 0x09,
    AGENT_ACTION_LOG_EVENT      = 0x0A,
    AGENT_ACTION_PERSIST_STATE  = 0x0B
} agent_action_id_t;

/* ---------- Execution API (FROZEN) ---------- */
agent_status_t  agent_action_execute(const agent_decision_t *decision);

/* ---------- Individual Actions (FROZEN) ---------- */
agent_status_t  agent_action_set_mode(agent_control_mode_t mode);
agent_status_t  agent_action_ai_trigger(void);
agent_status_t  agent_action_ai_abort(void);
agent_status_t  agent_action_ekf_reset(void);
agent_status_t  agent_action_emergency(void);
agent_status_t  agent_action_emergency_hold(void);
agent_status_t  agent_action_landing_step(void);
agent_status_t  agent_action_ipc_send(uint32_t msg_id, uint32_t payload);
agent_status_t  agent_action_ground_alert(uint32_t code);
agent_status_t  agent_action_log_event(uint32_t code,
                                       uint32_t arg1,
                                       uint32_t arg2);
agent_status_t  agent_action_persist_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_ACTIONS_H */