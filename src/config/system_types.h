/* ============================================================
 * SYSTEM TYPES — UNIFIED TYPE DEFINITIONS
 * ============================================================ */

#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "project_config.h"

/* ---------- Agent types (single source of truth) ---------- */
#include "agent_types.h"

/* ============================================================
 * TYPE ALIASES — Zephyr layer ↔ Agent layer
 * ============================================================ */

typedef agent_sensor_sample_t   sensor_sample_t;
typedef agent_sensor_sample_t   imu_sample_t;
typedef agent_ekf_state_t       ekf_state_t;
typedef agent_control_out_t     control_out_t;
typedef agent_control_mode_t    control_mode_t;

/* ============================================================
 * SECTION 1: SENSOR PRIMITIVES
 * ============================================================ */

typedef struct __attribute__((packed, aligned(4))) {
    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;
    uint8_t  health;
    uint8_t  reserved[3];
    uint32_t timestamp_us;
} baro_sample_t;

typedef struct __attribute__((packed, aligned(4))) {
    double   latitude_deg;
    double   longitude_deg;
    float    altitude_m;
    float    velocity_ned[3];
    float    hdop;
    uint8_t  satellites;
    uint8_t  fix_type;
    uint8_t  health;
    uint8_t  reserved;
    uint32_t timestamp_us;
} gps_sample_t;

typedef struct __attribute__((packed, aligned(4))) {
    float    mx;
    float    my;
    float    mz;
    uint8_t  health;
    uint8_t  reserved[3];
    uint32_t timestamp_us;
} mag_sample_t;

/* ============================================================
 * SECTION 2: AI DATA
 * ============================================================ */

typedef struct __attribute__((packed, aligned(4))) {
    int8_t   data[CFG_AI_INPUT_BYTES];
    uint32_t sequence_id;
    uint32_t timestamp_us;
} ai_input_t;

typedef struct __attribute__((packed, aligned(4))) {
    int8_t   data[CFG_AI_INPUT_BYTES];
    uint32_t sequence_id;
    uint32_t timestamp_us;
    uint32_t inference_us;
    uint8_t  valid;
    uint8_t  reserved[3];
} ai_output_t;

typedef struct {
    int8_t   buffer[CFG_AI_INPUT_BYTES];
    uint16_t write_idx;
    uint16_t count;
    uint32_t total_samples;
} ai_feed_cache_t;

/* ============================================================
 * SECTION 3: HEALTH
 * ============================================================ */

typedef enum {
    HEALTH_SENSOR_IMU       = 0,
    HEALTH_SENSOR_BARO      = 1,
    HEALTH_SENSOR_GPS       = 2,
    HEALTH_SENSOR_MAG       = 3,
    HEALTH_SENSOR_AI        = 4,
    HEALTH_SENSOR_EKF       = 5,
    HEALTH_SENSOR_CONTROL   = 6,
    HEALTH_SENSOR_POWER     = 7,
    HEALTH_SENSOR_COUNT     = 8
} health_sensor_id_t;

typedef struct {
    health_sensor_id_t  sensor;
    agent_health_t      health;
    agent_health_t      previous;
    uint32_t            fail_count;
    uint32_t            ok_count;
    uint32_t            last_update_us;
    uint32_t            last_fail_us;
    uint32_t            last_recover_us;
    uint32_t            total_fails;
    uint32_t            total_recovers;
} health_channel_t;

/* ============================================================
 * SECTION 4: COMMUNICATION (IPC)
 * ============================================================ */

typedef enum {
    IPC_MSG_NONE       = 0x0000,
    IPC_MSG_AI_INPUT   = 0x0001,
    IPC_MSG_AI_OUTPUT  = 0x0002,
    IPC_MSG_HEARTBEAT  = 0x0003,
    IPC_MSG_FAULT      = 0x0004,
    IPC_MSG_COUNT      = 5
} ipc_msg_type_t;

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t  sequence_id;
    uint32_t  timestamp_us;
    uint32_t  crc32;
    uint32_t  flags;
    int8_t    data[CFG_AI_INPUT_BYTES];
} ipc_ai_msg_t;

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t  cm4_alive;
    uint32_t  cm4_last_heartbeat_us;
    uint32_t  cm4_ai_inference_count;
    uint32_t  cm4_ai_latency_us;
    uint32_t  cm4_ai_latency_max_us;
    uint32_t  cm4_fault_count;
    uint32_t  reserved[8];
} ipc_status_t;

/* ============================================================
 * SECTION 5: ROS MESSAGES
 * ============================================================ */

typedef struct __attribute__((packed, aligned(4))) {
    float    pos[3];
    float    vel[3];
    float    quat[4];
    uint32_t timestamp_us;
} ros_pose_msg_t;

typedef struct __attribute__((packed, aligned(4))) {
    float    ax;
    float    ay;
    float    az;
    float    gx;
    float    gy;
    float    gz;
    uint32_t timestamp_us;
} ros_imu_msg_t;

typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  imu_health;
    uint8_t  ai_health;
    uint8_t  ekf_health;
    uint8_t  control_health;
    uint8_t  system_health;
    uint8_t  agent_state;
    uint16_t reserved;
    uint32_t fault_count;
    uint32_t timestamp_us;
} ros_health_msg_t;

/* ============================================================
 * SECTION 6: RUNTIME METRICS
 * ============================================================ */

typedef struct {
    uint32_t loop_count;
    uint32_t loop_wcet_us;
    uint32_t loop_wcet_max_us;
    uint32_t deadline_miss_count;
    uint32_t wcet_violation_count;
    uint32_t cpu_load_pct_x100;
} runtime_metrics_t;

/* ============================================================
 * SECTION 7: COMPILE-TIME VALIDATION
 * ============================================================ */

_Static_assert(sizeof(baro_sample_t) <= 32, "baro_sample_t too large");
_Static_assert(sizeof(gps_sample_t) <= 64, "gps_sample_t too large");
_Static_assert(sizeof(mag_sample_t) <= 32, "mag_sample_t too large");
_Static_assert(sizeof(ipc_ai_msg_t) == CFG_AI_INPUT_BYTES + 16,
               "ipc_ai_msg_t size mismatch");
_Static_assert(sizeof(ros_pose_msg_t) == 44, "ros_pose_msg_t size mismatch");
_Static_assert(sizeof(ros_imu_msg_t) == 28, "ros_imu_msg_t size mismatch");

_Static_assert(HEALTH_SENSOR_COUNT == 8, "health_sensor_id_t count mismatch");
_Static_assert(IPC_MSG_COUNT == 5, "ipc_msg_type_t count mismatch");

#endif /* SYSTEM_TYPES_H */