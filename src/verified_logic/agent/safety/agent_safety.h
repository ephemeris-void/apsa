#ifndef AGENT_SAFETY_H
#define AGENT_SAFETY_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

agent_status_t  agent_safety_init(void);
bool            agent_safety_check(void);
agent_status_t  agent_safety_self_test(void);
void            agent_safety_escalate(void);
void            agent_safety_deescalate(void);
bool            agent_safety_can_land(void);
bool            agent_safety_can_land_standalone(void);
bool            agent_safety_is_landed(void);

float           agent_safety_envelope_accel_g(void);
float           agent_safety_envelope_gyro_dps(void);
float           agent_safety_envelope_alt_max_m(void);
float           agent_safety_envelope_vel_max_ms(void);

#ifdef __cplusplus
}
#endif

#endif