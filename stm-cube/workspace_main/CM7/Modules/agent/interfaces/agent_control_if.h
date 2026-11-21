#ifndef AGENT_CONTROL_IF_H
#define AGENT_CONTROL_IF_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * CONTROL INTERFACE — FROZEN
 * ============================================================
 * External module: MPC / Control (Modules/control/)
 * Link point:      mpc.c / mpc.h
 * Controller:      Model Predictive Control (or PID/LQR)
 * Board:           NUCLEO-H755ZI-Q
 *
 * To replace MPC with PID / LQR / other:
 *   1. Implement these functions in new controller
 *   2. This interface stays UNCHANGED
 *   3. Agent code stays UNCHANGED
 *
 * WCET budget:
 *   control_init    : 200 µs (boot only)
 *   control_update  : 500 µs
 *   control_get_out : 5 µs
 *   control_set_mode: 1 µs
 *   control_health  : 1 µs
 *
 * NOTE: agent_control_mode_t is defined in agent_types.h
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  control_init(void);

/* ---------- Update (Compute actuator output) ---------- */
agent_status_t  control_update(const agent_ekf_state_t *state);

/* ---------- Get Last Output ---------- */
agent_status_t  control_get_output(agent_control_out_t *out);

/* ---------- Set Control Mode ---------- */
agent_status_t  control_set_mode(agent_control_mode_t mode);

/* ---------- Health ---------- */
agent_status_t  control_health(void);

/* ---------- Failure Injection ---------- */
void            control_on_failure(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_CONTROL_IF_H */