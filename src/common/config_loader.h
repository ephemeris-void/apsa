/* ============================================================
 * CONFIG LOADER — RUNTIME CONFIGURATION
 * ============================================================
 * - Load tunable parameters at boot (from flash or defaults)
 * - Override compile-time CFG_* values where needed
 * - CRC-validated flash storage
 * - Safe rollback to defaults on corruption
 * - Extensible: add parameter without breaking existing code
 * - Version-aware: rejects incompatible configs
 *
 * NOTE: FLASH_SIZE renamed to CFG_FLASH_SIZE to avoid conflict
 *       with Zephyr's own CONFIG_FLASH_SIZE.
 * ============================================================ */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Config Magic + Version
 * ============================================================ */
#define CONFIG_MAGIC          0xC0FFEE42UL
#define CONFIG_VERSION        1U
#define CONFIG_FLASH_SECTOR   7U
#define CONFIG_FLASH_ADDR     0x080E0000UL
#define CFG_FLASH_SIZE        0x20000UL  /* 128 KB */

/* ============================================================
 * Runtime Tunable Parameters
 * ============================================================ */
typedef struct __attribute__((packed, aligned(4))) {
    /* Header */
    uint32_t magic;
    uint16_t version;
    uint16_t reserved0;

    /* Timing */
    uint32_t agent_loop_period_us;
    uint32_t ekf_zupt_accel_tol_x1000;
    uint32_t ekf_zupt_gyro_tol_x1000;
    uint16_t ekf_zupt_min_samples;
    uint16_t ekf_accel_decimation;

    /* Health thresholds */
    uint8_t  health_fail_confirm;
    uint8_t  health_recover_confirm;
    uint8_t  imu_fail_confirm;
    uint8_t  imu_recover_confirm;

    /* AI */
    uint16_t ai_trigger_period_ms;
    uint16_t ai_max_latency_ms;
    uint16_t ai_confidence_min_x1000;

    /* ROS */
    uint16_t ros_publish_period_ms;
    uint16_t ros_health_period_ms;

    /* Control */
    float    control_max_thrust_n;
    float    control_max_roll_rad;
    float    control_max_pitch_rad;
    float    control_max_yaw_rad;

    /* Envelope */
    float    envelope_alt_max_m;
    float    envelope_vel_max_ms;

    /* Integrity */
    uint32_t crc32;

    /* Padding to fixed size (extensible) */
    uint32_t reserved1[16];
} runtime_config_t;

_Static_assert(sizeof(runtime_config_t) <= 256,
               "runtime_config_t exceeds flash slot");

/* ============================================================
 * API
 * ============================================================ */

proj_status_t config_loader_init(void);
const runtime_config_t *config_get(void);
void config_set_defaults(void);
proj_status_t config_save(void);
proj_status_t config_load(void);
proj_status_t config_factory_reset(void);
proj_status_t config_validate(const runtime_config_t *cfg);
void config_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LOADER_H */