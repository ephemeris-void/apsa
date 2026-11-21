#ifndef AGENT_HEALTH_H
#define AGENT_HEALTH_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT HEALTH — FROZEN
 * ============================================================
 * Per-sensor health tracking with confirm counters.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per call.
 * ============================================================ */

#define AGENT_HEALTH_MAX_SENSORS        8U
#define AGENT_HEALTH_FAIL_CONFIRM       3U
#define AGENT_HEALTH_RECOVER_CONFIRM    10U
#define AGENT_HEALTH_HISTORY_DEPTH      8U

/* ---------- Sensor Channel ---------- */
typedef enum {
    AGENT_HEALTH_SENSOR_IMU         = 0,
    AGENT_HEALTH_SENSOR_BARO        = 1,
    AGENT_HEALTH_SENSOR_GPS         = 2,
    AGENT_HEALTH_SENSOR_MAG         = 3,
    AGENT_HEALTH_SENSOR_AI          = 4,
    AGENT_HEALTH_SENSOR_EKF         = 5,
    AGENT_HEALTH_SENSOR_CONTROL     = 6,
    AGENT_HEALTH_SENSOR_POWER       = 7,
    AGENT_HEALTH_SENSOR_COUNT       = 8
} agent_health_sensor_t;

/* ---------- Per-Channel State ---------- */
typedef struct {
    agent_health_sensor_t   sensor;
    agent_health_t          health;
    agent_health_t          previous;
    uint32_t                fail_count;
    uint32_t                ok_count;
    uint32_t                last_update_us;
    uint32_t                last_fail_us;
    uint32_t                last_recover_us;
    uint32_t                total_fails;
    uint32_t                total_recovers;
} agent_health_channel_t;

/* ---------- Aggregate State ---------- */
typedef struct {
    agent_health_channel_t  channels[AGENT_HEALTH_MAX_SENSORS];
    agent_health_t          system;
    uint32_t                last_update_us;
    uint32_t                worst_sensor;
    uint32_t                reserved[2];
} agent_health_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_health_init(void);
agent_status_t                  agent_health_reset(void);
agent_status_t                  agent_health_update(void);

agent_status_t                  agent_health_set(agent_health_sensor_t sensor,
                                                 agent_health_t health);

agent_health_t                  agent_health_get(agent_health_sensor_t sensor);
agent_health_t                  agent_health_system(void);

const agent_health_channel_t   *agent_health_channel(agent_health_sensor_t sensor);
const agent_health_state_t     *agent_health_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_HEALTH_H */