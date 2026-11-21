#ifndef AGENT_TYPES_H
#define AGENT_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT TYPES — FROZEN
 * ============================================================
 * Safety-critical definitions. Never change shape or size.
 * Any modification requires re-verification (DO-178C level).
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 *
 * Layout rules:
 *   - All enums are int32 (ABI stable)
 *   - All structs are packed-aligned
 *   - reserved[] fields for future extension (ABI safe)
 * ============================================================ */

/* ---------- State Enum ---------- */
typedef enum {
    AGENT_STATE_BOOT        = 0,
    AGENT_STATE_MONITOR     = 1,
    AGENT_STATE_DEGRADED    = 2,
    AGENT_STATE_VIRTUALIZE  = 3,
    AGENT_STATE_FUSION      = 4,
    AGENT_STATE_RECOVERY    = 5,
    AGENT_STATE_EMERGENCY   = 6,
    AGENT_STATE_LANDING     = 7,
    AGENT_STATE_MAX         = 8
} agent_state_t;

/* ---------- Health Enum ---------- */
typedef enum {
    AGENT_HEALTH_UNKNOWN    = 0,
    AGENT_HEALTH_OK         = 1,
    AGENT_HEALTH_DEGRADED   = 2,
    AGENT_HEALTH_FAIL       = 3
} agent_health_t;

/* ---------- Priority Enum ---------- */
typedef enum {
    AGENT_PRIO_LOW          = 0,
    AGENT_PRIO_NORMAL       = 1,
    AGENT_PRIO_HIGH         = 2,
    AGENT_PRIO_CRITICAL     = 3
} agent_priority_t;

/* ---------- AI Status ---------- */
typedef enum {
    AI_STATUS_IDLE          = 0,
    AI_STATUS_RUNNING       = 1,
    AI_STATUS_READY         = 2,
    AI_STATUS_ERROR         = 3
} ai_status_t;

/* ---------- Control Mode ---------- */
typedef enum {
    AGENT_MODE_NORMAL       = 0,
    AGENT_MODE_DEGRADED     = 1,
    AGENT_MODE_VIRTUAL      = 2,
    AGENT_MODE_FUSION       = 3,
    AGENT_MODE_RECOVERY     = 4,
    AGENT_MODE_EMERGENCY    = 5,
    AGENT_MODE_LANDING      = 6
} agent_control_mode_t;

/* ---------- Sensor Sample ---------- */
typedef struct {
    int16_t        ax;
    int16_t        ay;
    int16_t        az;
    int16_t        gx;
    int16_t        gy;
    int16_t        gz;
    uint32_t       timestamp_us;
    agent_health_t health;
    uint32_t       reserved[2];    /* future: temperature, pressure */
} agent_sensor_sample_t;

/* ---------- EKF State ---------- */
typedef struct {
    float    pos[3];
    float    vel[3];
    float    quat[4];
    float    gyro_bias[3];
    float    accel_bias[3];
    float    confidence;
    uint32_t timestamp_us;
    uint32_t reserved[2];          /* future: mag_bias, wind */
} agent_ekf_state_t;

/* ---------- Control Output ---------- */
typedef struct {
    float    thrust;
    float    roll;
    float    pitch;
    float    yaw;
    uint32_t timestamp_us;
    uint32_t reserved[2];          /* future: aux channels */
} agent_control_out_t;

/* ---------- Decision ---------- */
typedef struct {
    agent_state_t    next_state;
    agent_priority_t priority;
    uint8_t          action_id;
    uint32_t         reason_code;
    uint32_t         timestamp_us;
    uint32_t         reserved[2];  /* future: confidence, source */
} agent_decision_t;

/* ---------- Action ---------- */
typedef struct {
    uint8_t  action_id;
    void    *payload;
    uint32_t payload_size;
    uint32_t deadline_us;
    uint32_t timestamp_us;
    uint32_t reserved[2];
} agent_action_t;

/* ---------- Agent Context (basic) ---------- */
typedef struct {
    agent_state_t    current_state;
    agent_state_t    previous_state;
    uint32_t         time_in_state_us;
    uint32_t         state_entry_us;
    uint32_t         boot_time_us;
    uint32_t         iteration_count;
    uint32_t         fault_count;
    uint32_t         recovery_count;
    agent_priority_t current_priority;
    agent_health_t   system_health;
    uint32_t         reserved[4];  /* future: version, flags */
} agent_context_t;

/* ---------- Fault Codes ---------- */
typedef enum {
    AGENT_FAULT_NONE          = 0x0000,
    AGENT_FAULT_IMU_FAIL      = 0x0001,
    AGENT_FAULT_BARO_FAIL     = 0x0002,
    AGENT_FAULT_GPS_FAIL      = 0x0003,
    AGENT_FAULT_AI_FAIL       = 0x0004,
    AGENT_FAULT_EKF_DIVERGE   = 0x0005,
    AGENT_FAULT_DEADLINE_MISS = 0x0006,
    AGENT_FAULT_WCET_VIOLATE  = 0x0007,
    AGENT_FAULT_IPC_TIMEOUT   = 0x0008,
    AGENT_FAULT_STACK_OVER    = 0x0009,
    AGENT_FAULT_HARDFAULT     = 0x000A,
    AGENT_FAULT_WATCHDOG      = 0x000B
} agent_fault_t;

/* ---------- Reason Codes ---------- */
typedef enum {
    AGENT_REASON_NONE         = 0x00,
    AGENT_REASON_INNOVATION   = 0x01,
    AGENT_REASON_CROSSCHECK   = 0x02,
    AGENT_REASON_TIMEOUT      = 0x03,
    AGENT_REASON_RANGE        = 0x04,
    AGENT_REASON_DRIFT        = 0x05,
    AGENT_REASON_CONFIDENCE   = 0x06,
    AGENT_REASON_ENVELOPE     = 0x07,
    AGENT_REASON_WCET         = 0x08
} agent_reason_t;

/* ---------- Return Codes ---------- */
typedef enum {
    AGENT_OK                  = 0,
    AGENT_ERR                 = -1,
    AGENT_ERR_TIMEOUT         = -2,
    AGENT_ERR_NOT_INIT        = -3,
    AGENT_ERR_INVALID_STATE   = -4,
    AGENT_ERR_WCET            = -5,
    AGENT_ERR_HW              = -6
} agent_status_t;

#ifdef __cplusplus
}
#endif

#endif /* AGENT_TYPES_H */