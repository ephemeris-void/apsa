#ifndef APSA_BRIDGE_H
#define APSA_BRIDGE_H

#include "../../agent/core/agent_types.h"
#include "../../agent/interfaces/agent_ekf_if.h"
#include "../../agent/interfaces/agent_imu_if.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef APSA_USE_ESKF
#define APSA_USE_ESKF               1
#endif
#ifndef APSA_USE_INNOVATION_MON
#define APSA_USE_INNOVATION_MON     1
#endif
#ifndef APSA_USE_SENSOR_DELAY
#define APSA_USE_SENSOR_DELAY       0
#endif
#ifndef APSA_USE_EKF_BACKUP
#define APSA_USE_EKF_BACKUP         0
#endif
#ifndef APSA_USE_UKF_BACKUP
#define APSA_USE_UKF_BACKUP         0
#endif
#ifndef APSA_USE_FRAME
#define APSA_USE_FRAME              0
#endif
#ifndef APSA_USE_GRAVITY
#define APSA_USE_GRAVITY            1
#endif

#ifndef APSA_USE_LQR
#define APSA_USE_LQR                1
#endif
#ifndef APSA_USE_SMC
#define APSA_USE_SMC                1
#endif
#ifndef APSA_USE_EXTENDED_SMC
#define APSA_USE_EXTENDED_SMC       0
#endif
#ifndef APSA_USE_MPC
#define APSA_USE_MPC                0
#endif
#ifndef APSA_USE_ALLOCATION
#define APSA_USE_ALLOCATION         0
#endif
#ifndef APSA_USE_BUMPLESS
#define APSA_USE_BUMPLESS           1
#endif
#ifndef APSA_USE_RIGID_BODY
#define APSA_USE_RIGID_BODY         1
#endif

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

const apsa_bridge_ekf_status_t  *apsa_bridge_ekf_status(void);
const apsa_bridge_ctrl_status_t *apsa_bridge_ctrl_status(void);

void apsa_bridge_reset_all(void);

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