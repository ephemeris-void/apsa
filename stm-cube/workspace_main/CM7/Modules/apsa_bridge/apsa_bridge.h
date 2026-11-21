#ifndef APSA_BRIDGE_H
#define APSA_BRIDGE_H

#include "../agent/core/agent_types.h"
#include "../agent/interfaces/agent_ekf_if.h"
#include "../agent/interfaces/agent_imu_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * APSA BRIDGE — Adapter Layer
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 *
 * Purpose:
 *   Provides ekf_* and control_* interface to the 25 agent files,
 *   while using apsa_modules (20 C++ headers) internally.
 *
 *   If apsa changes tomorrow, only the adapter files change.
 *   The 25 agent files and the frozen interface headers NEVER change.
 *
 * Layers:
 *   25 agent files  →  ekf_* / control_*  (frozen interface)
 *                          ↓
 *                     apsa_bridge/ (this adapter)
 *                          ↓
 *                     20 apsa_modules (C++ headers)
 *
 * Feature Flags:
 *   Comment / uncomment in apsa_bridge.h or override at build time.
 *   Active flag = apsa function used.
 *   Inactive flag = fallback to internal PID / simple logic.
 * ============================================================ */

/* ============================================================
 * FEATURE FLAGS — activate apsa modules when needed
 * ============================================================ */

/* -------- Estimation -------- */
#ifndef APSA_USE_ESKF
#define APSA_USE_ESKF               1   /* 19-state ESKF (active) */
#endif

#ifndef APSA_USE_INNOVATION_MON
#define APSA_USE_INNOVATION_MON     1   /* NIS/NEES IMU health (active) */
#endif

#ifndef APSA_USE_SENSOR_DELAY
#define APSA_USE_SENSOR_DELAY       0   /* delayed measurements */
#endif

#ifndef APSA_USE_EKF_BACKUP
#define APSA_USE_EKF_BACKUP         0   /* EKF fallback */
#endif

#ifndef APSA_USE_UKF_BACKUP
#define APSA_USE_UKF_BACKUP         0   /* UKF fallback */
#endif

#ifndef APSA_USE_FRAME
#define APSA_USE_FRAME              0   /* GCRS<->ITRS<->NED */
#endif

#ifndef APSA_USE_GRAVITY
#define APSA_USE_GRAVITY            0   /* J2/J3/J4 gravity */
#endif

/* -------- Control -------- */
#ifndef APSA_USE_LQR
#define APSA_USE_LQR                0   /* LQR position control */
#endif

#ifndef APSA_USE_SMC
#define APSA_USE_SMC                0   /* SMC attitude control */
#endif

#ifndef APSA_USE_EXTENDED_SMC
#define APSA_USE_EXTENDED_SMC       0   /* super-twisting SMC */
#endif

#ifndef APSA_USE_MPC
#define APSA_USE_MPC                0   /* Tube MPC */
#endif

#ifndef APSA_USE_ALLOCATION
#define APSA_USE_ALLOCATION         0   /* motor mixing */
#endif

#ifndef APSA_USE_BUMPLESS
#define APSA_USE_BUMPLESS           1   /* smooth mode switch (active) */
#endif

#ifndef APSA_USE_RIGID_BODY
#define APSA_USE_RIGID_BODY         0   /* 6DOF dynamics */
#endif

/* ============================================================
 * ADAPTER STATUS — read-only diagnostics
 * ============================================================ */

typedef struct {
    uint32_t update_count;
    uint32_t last_update_us;
    float    confidence;
    bool     diverged;
    bool     initialized;
} apsa_bridge_ekf_status_t;

typedef struct {
    uint32_t iteration;
    uint32_t solve_us;
    uint32_t solve_us_max;
    bool     failed;
    bool     initialized;
} apsa_bridge_ctrl_status_t;

/* ============================================================
 * STATUS ACCESSORS
 * ============================================================ */

const apsa_bridge_ekf_status_t  *apsa_bridge_ekf_status(void);
const apsa_bridge_ctrl_status_t *apsa_bridge_ctrl_status(void);

/* ============================================================
 * RESET BOTH SIDES (ekf + control)
 * ============================================================ */

void apsa_bridge_reset_all(void);

/* ============================================================
 * EKF ADAPTER DECLARATIONS
 * ============================================================
 * These are implemented in apsa_ekf_adapter.cpp
 * (public API from agent_ekf_if.h so the .cpp definition matches)
 * ============================================================ */

agent_status_t ekf_init(void);
agent_status_t ekf_update(const agent_sensor_sample_t *sample);
agent_status_t ekf_predict(float dt);
agent_status_t ekf_get_state(agent_ekf_state_t *state);
agent_status_t ekf_health(void);
void           ekf_on_failure(void);
bool           ekf_has_diverged(void);

#ifdef __cplusplus
}
#endif

#endif /* APSA_BRIDGE_H */