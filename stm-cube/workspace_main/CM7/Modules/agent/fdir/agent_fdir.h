#ifndef AGENT_FDIR_H
#define AGENT_FDIR_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT FDIR — FROZEN
 * ============================================================
 * Fault Detection, Isolation, Recovery.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 80 µs max per update.
 * ============================================================ */

#define AGENT_FDIR_MAX_SENSORS      8U
#define AGENT_FDIR_HISTORY_DEPTH    8U

/* ---------- Sensor Channels ---------- */
typedef enum {
    AGENT_FDIR_SENSOR_IMU        = 0,
    AGENT_FDIR_SENSOR_BARO       = 1,
    AGENT_FDIR_SENSOR_GPS        = 2,
    AGENT_FDIR_SENSOR_MAG        = 3,
    AGENT_FDIR_SENSOR_AI         = 4,
    AGENT_FDIR_SENSOR_EKF        = 5,
    AGENT_FDIR_SENSOR_CONTROL    = 6,
    AGENT_FDIR_SENSOR_COUNT      = 7
} agent_fdir_sensor_t;

/* ---------- Fault Types ---------- */
typedef enum {
    AGENT_FDIR_FAULT_NONE        = 0,
    AGENT_FDIR_FAULT_DRIFT       = 1,
    AGENT_FDIR_FAULT_BIAS        = 2,
    AGENT_FDIR_FAULT_STUCK       = 3,
    AGENT_FDIR_FAULT_SPIKE       = 4,
    AGENT_FDIR_FAULT_TIMEOUT     = 5,
    AGENT_FDIR_FAULT_DROPOUT     = 6,
    AGENT_FDIR_FAULT_RANGE       = 7,
    AGENT_FDIR_FAULT_INNOVATION  = 8,
    AGENT_FDIR_FAULT_CROSSCHECK  = 9,
    AGENT_FDIR_FAULT_UNKNOWN     = 10
} agent_fdir_fault_t;

/* ---------- Channel State ---------- */
typedef enum {
    AGENT_FDIR_STATE_OK          = 0,
    AGENT_FDIR_STATE_SUSPECT     = 1,
    AGENT_FDIR_STATE_DEGRADED    = 2,
    AGENT_FDIR_STATE_FAILED      = 3,
    AGENT_FDIR_STATE_ISOLATED    = 4
} agent_fdir_state_t;

/* ---------- Channel ---------- */
typedef struct {
    agent_fdir_sensor_t  sensor;
    agent_fdir_state_t   state;
    agent_fdir_fault_t   fault;
    uint32_t             fault_count;
    uint32_t             ok_count;
    uint32_t             fault_first_us;
    uint32_t             fault_last_us;
    uint32_t             isolation_us;
    float                confidence;
    bool                 isolated;
    uint32_t             reserved[2];   /* ABI-safe future extension */
} agent_fdir_channel_t;

/* ---------- Status ---------- */
typedef struct {
    agent_fdir_channel_t channels[AGENT_FDIR_MAX_SENSORS];
    uint32_t             fault_total;
    uint32_t             isolate_total;
    uint32_t             recover_total;
    uint32_t             last_update_us;
    uint32_t             reserved[2];
} agent_fdir_status_t;

/* ---------- Event ---------- */
typedef struct {
    agent_fdir_sensor_t  sensor;
    agent_fdir_fault_t   fault;
    uint32_t             reason;
    uint32_t             timestamp_us;
} agent_fdir_event_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_fdir_init(void);
agent_status_t              agent_fdir_reset(void);
agent_status_t              agent_fdir_update(void);

agent_fdir_state_t          agent_fdir_get_state(agent_fdir_sensor_t s);
agent_fdir_fault_t          agent_fdir_get_fault(agent_fdir_sensor_t s);
float                       agent_fdir_get_confidence(agent_fdir_sensor_t s);
bool                        agent_fdir_is_isolated(agent_fdir_sensor_t s);

agent_status_t              agent_fdir_isolate(agent_fdir_sensor_t s);
agent_status_t              agent_fdir_restore(agent_fdir_sensor_t s);

const agent_fdir_status_t  *agent_fdir_status(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_FDIR_H */