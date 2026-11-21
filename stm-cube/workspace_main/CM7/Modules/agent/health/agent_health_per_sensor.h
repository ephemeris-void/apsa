#ifndef AGENT_HEALTH_PER_SENSOR_H
#define AGENT_HEALTH_PER_SENSOR_H

#include "agent_health.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT HEALTH — PER-SENSOR POLLING
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 25 µs max per poll (all sensors)
 * ============================================================ */

agent_status_t  agent_health_per_sensor_init(void);

agent_status_t  agent_health_poll_imu(void);
agent_status_t  agent_health_poll_ai(void);
agent_status_t  agent_health_poll_ekf(void);
agent_status_t  agent_health_poll_control(void);
agent_status_t  agent_health_poll_all(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_HEALTH_PER_SENSOR_H */