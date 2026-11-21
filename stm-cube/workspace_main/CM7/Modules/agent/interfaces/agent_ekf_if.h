#ifndef AGENT_EKF_IF_H
#define AGENT_EKF_IF_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

agent_status_t  ekf_init(void);

void            ekf_set_initial_attitude_from_accel(float ax_ms2,
                                                     float ay_ms2,
                                                     float az_ms2);

agent_status_t  ekf_update(const agent_sensor_sample_t *sample);

agent_status_t  ekf_predict(float dt);

agent_status_t  ekf_get_state(agent_ekf_state_t *state);

agent_status_t  ekf_health(void);

void            ekf_on_failure(void);

bool            ekf_has_diverged(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_EKF_IF_H */