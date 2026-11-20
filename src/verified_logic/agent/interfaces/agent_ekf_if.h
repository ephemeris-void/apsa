#ifndef AGENT_EKF_IF_H
#define AGENT_EKF_IF_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * EKF / ESKF INTERFACE — FROZEN
 * ============================================================
 * External module: ESKF (Modules/eskf/)
 * Link point:      eskf.c / eskf.h
 * Filter:          Error-State Kalman Filter (16 states)
 * Board:           NUCLEO-H755ZI-Q
 *
 * To replace EKF with UKF / IEKF:
 *   1. Implement these functions in new filter
 *   2. This interface stays UNCHANGED
 *   3. Agent code stays UNCHANGED
 *
 * WCET budget:
 *   ekf_init        : 200 µs (boot only)
 *   ekf_update      : 100 µs
 *   ekf_predict     : 50 µs
 *   ekf_get_state   : 5 µs
 *   ekf_health      : 1 µs
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  ekf_init(void);

/* ---------- Update (Fuse sensor sample) ----------
 * Called every iteration with sensor data. */
agent_status_t  ekf_update(const agent_sensor_sample_t *sample);

/* ---------- Predict (Propagate state) ---------- */
agent_status_t  ekf_predict(float dt);

/* ---------- Get Current State ---------- */
agent_status_t  ekf_get_state(agent_ekf_state_t *state);

/* ---------- Health ---------- */
agent_status_t  ekf_health(void);

/* ---------- Failure Injection ---------- */
void            ekf_on_failure(void);

/* ---------- Divergence Check ---------- */
bool            ekf_has_diverged(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_EKF_IF_H */