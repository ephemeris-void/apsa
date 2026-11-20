/* ============================================================
 * CONFIG LOADER — IMPLEMENTATION
 * ============================================================ */

#include "config_loader.h"
#include "logger.h"
#include "error_handler.h"
#include "../config/project_config.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <string.h>

/* ============================================================
 * Internal State
 * ============================================================ */
static runtime_config_t g_active_config;
static bool             g_initialized = false;

/* ============================================================
 * CRC32 (bitwise)
 * ============================================================ */
static uint32_t config_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (uint32_t b = 0U; b < 8U; b++) {
            if (crc & 1U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ============================================================
 * Flash access via Zephyr flash_map
 * ============================================================ */
#if FIXED_PARTITION_EXISTS(config_storage)
#define CONFIG_PARTITION_ID   FIXED_PARTITION_ID(config_storage)
#define CONFIG_FLASH_READY    1
#else
#define CONFIG_FLASH_READY    0
#endif

static const struct device *config_flash_dev = NULL;
static off_t                 config_flash_offset = 0;

static proj_status_t config_flash_open(void)
{
#if CONFIG_FLASH_READY
    config_flash_dev = FIXED_PARTITION_DEVICE(config_storage);
    config_flash_offset = FIXED_PARTITION_OFFSET(config_storage);

    if (config_flash_dev == NULL || !device_is_ready(config_flash_dev)) {
        return PROJ_ERR_HW;
    }
    return PROJ_OK;
#else
    return PROJ_ERR_NOT_FOUND;
#endif
}

static proj_status_t config_flash_read(void *buf, size_t len)
{
#if CONFIG_FLASH_READY
    if (config_flash_dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }
    int rc = flash_read(config_flash_dev, config_flash_offset, buf, len);
    return (rc == 0) ? PROJ_OK : PROJ_ERR_HW;
#else
    (void)buf; (void)len;
    return PROJ_ERR_NOT_FOUND;
#endif
}

static proj_status_t config_flash_write(const void *buf, size_t len)
{
#if CONFIG_FLASH_READY
    if (config_flash_dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    /* Erase first */
    struct flash_pages_info info;
    int rc = flash_get_page_info_by_offs(config_flash_dev,
                                          config_flash_offset,
                                          &info);
    if (rc != 0) {
        return PROJ_ERR_HW;
    }

    rc = flash_erase(config_flash_dev, config_flash_offset, info.size);
    if (rc != 0) {
        return PROJ_ERR_HW;
    }

    rc = flash_write(config_flash_dev, config_flash_offset, buf, len);
    return (rc == 0) ? PROJ_OK : PROJ_ERR_HW;
#else
    (void)buf; (void)len;
    return PROJ_ERR_NOT_FOUND;
#endif
}

/* ============================================================
 * DEFAULTS
 * ============================================================ */
void config_set_defaults(void)
{
    memset(&g_active_config, 0, sizeof(g_active_config));

    g_active_config.magic   = CONFIG_MAGIC;
    g_active_config.version = CONFIG_VERSION;

    /* Timing */
    g_active_config.agent_loop_period_us    = CFG_AGENT_THREAD_PERIOD_US;
    g_active_config.ekf_zupt_accel_tol_x1000 = (uint32_t)(CFG_EKF_ZUPT_ACCEL_TOL * 1000.0f);
    g_active_config.ekf_zupt_gyro_tol_x1000  = (uint32_t)(CFG_EKF_ZUPT_GYRO_TOL  * 1000.0f);
    g_active_config.ekf_zupt_min_samples     = (uint16_t)CFG_EKF_ZUPT_MIN_SAMPLES;
    g_active_config.ekf_accel_decimation     = (uint16_t)CFG_EKF_ACCEL_DECIMATION;

    /* Health */
    g_active_config.health_fail_confirm     = (uint8_t)CFG_HEALTH_FAIL_CONFIRM;
    g_active_config.health_recover_confirm  = (uint8_t)CFG_HEALTH_RECOVER_CONFIRM;
    g_active_config.imu_fail_confirm        = (uint8_t)CFG_IMU_FAIL_CONFIRM_COUNT;
    g_active_config.imu_recover_confirm     = (uint8_t)CFG_IMU_RECOVER_CONFIRM_COUNT;

    /* AI */
    g_active_config.ai_trigger_period_ms    = (uint16_t)CFG_AI_TRIGGER_PERIOD_MS;
    g_active_config.ai_max_latency_ms       = (uint16_t)(CFG_AI_MAX_LATENCY_US / 1000U);
    g_active_config.ai_confidence_min_x1000 = (uint16_t)(CFG_AI_CONFIDENCE_MIN * 1000.0f);

    /* ROS */
    g_active_config.ros_publish_period_ms   = (uint16_t)CFG_ROS_PUBLISH_PERIOD_MS;
    g_active_config.ros_health_period_ms    = 1000U;

    /* Control */
    g_active_config.control_max_thrust_n    = 50.0f;
    g_active_config.control_max_roll_rad    = 0.5f;
    g_active_config.control_max_pitch_rad   = 0.5f;
    g_active_config.control_max_yaw_rad     = 1.0f;

    /* Envelope */
    g_active_config.envelope_alt_max_m      = CFG_ENVELOPE_ALT_MAX_M;
    g_active_config.envelope_vel_max_ms     = CFG_ENVELOPE_VEL_MAX_MS;
}

/* ============================================================
 * VALIDATION
 * ============================================================ */
proj_status_t config_validate(const runtime_config_t *cfg)
{
    if (cfg == NULL) {
        return PROJ_ERR_NULL;
    }

    if (cfg->magic   != CONFIG_MAGIC)   return PROJ_ERR_INVALID;
    if (cfg->version != CONFIG_VERSION) return PROJ_ERR_INVALID;

    if (cfg->agent_loop_period_us == 0U)    return PROJ_ERR_INVALID;
    if (cfg->agent_loop_period_us > 100000U) return PROJ_ERR_INVALID;

    if (cfg->ekf_zupt_accel_tol_x1000 == 0U) return PROJ_ERR_INVALID;
    if (cfg->ekf_zupt_accel_tol_x1000 > 10000U) return PROJ_ERR_INVALID;

    if (cfg->ekf_zupt_min_samples == 0U) return PROJ_ERR_INVALID;
    if (cfg->ekf_zupt_min_samples > 1000U) return PROJ_ERR_INVALID;

    if (cfg->ai_trigger_period_ms == 0U) return PROJ_ERR_INVALID;
    if (cfg->ai_trigger_period_ms > 1000U) return PROJ_ERR_INVALID;

    if (cfg->control_max_thrust_n <= 0.0f) return PROJ_ERR_INVALID;
    if (cfg->control_max_thrust_n > 500.0f) return PROJ_ERR_INVALID;

    if (cfg->envelope_alt_max_m <= 0.0f)   return PROJ_ERR_INVALID;
    if (cfg->envelope_vel_max_ms <= 0.0f)  return PROJ_ERR_INVALID;

    return PROJ_OK;
}

/* ============================================================
 * LOAD
 * ============================================================ */
proj_status_t config_load(void)
{
    runtime_config_t tmp;
    memset(&tmp, 0, sizeof(tmp));

    proj_status_t st = config_flash_read(&tmp, sizeof(tmp));
    if (st != PROJ_OK) {
        return st;
    }

    if (config_validate(&tmp) != PROJ_OK) {
        return PROJ_ERR_INVALID;
    }

    /* Verify CRC (CRC computed over struct excluding crc field itself) */
    uint32_t stored_crc = tmp.crc32;
    tmp.crc32 = 0U;
    uint32_t computed = config_crc32((const uint8_t *)&tmp, sizeof(tmp));
    if (stored_crc != computed) {
        return PROJ_ERR_CHECKSUM;
    }

    memcpy(&g_active_config, &tmp, sizeof(tmp));
    return PROJ_OK;
}

/* ============================================================
 * SAVE
 * ============================================================ */
proj_status_t config_save(void)
{
    runtime_config_t tmp;
    memcpy(&tmp, &g_active_config, sizeof(tmp));

    /* Zero CRC before computing */
    tmp.crc32 = 0U;
    tmp.crc32 = config_crc32((const uint8_t *)&tmp, sizeof(tmp));

    return config_flash_write(&tmp, sizeof(tmp));
}

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t config_loader_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    /* 1. Set defaults first (safe fallback) */
    config_set_defaults();

    /* 2. Try to open flash */
    proj_status_t st = config_flash_open();
    if (st == PROJ_OK) {
        /* 3. Try to load stored config */
        st = config_load();
        if (st != PROJ_OK) {
            /* Corrupted or missing → save defaults */
            LOG_W(LOG_TAG_MAIN, "config: load failed (%d), using defaults", (int)st);
            (void)config_save();
        }
    } else {
        LOG_W(LOG_TAG_MAIN, "config: no flash partition, using defaults");
    }

    g_initialized = true;
    return PROJ_OK;
}

/* ============================================================
 * ACCESSOR
 * ============================================================ */
const runtime_config_t *config_get(void)
{
    return &g_active_config;
}

/* ============================================================
 * FACTORY RESET
 * ============================================================ */
proj_status_t config_factory_reset(void)
{
    config_set_defaults();
    return config_save();
}

/* ============================================================
 * DUMP
 * ============================================================ */
void config_dump(void)
{
    LOG_I(LOG_TAG_MAIN, "=== CONFIG DUMP ===");
    LOG_I(LOG_TAG_MAIN, "  loop_period_us       : %u",
          (unsigned)g_active_config.agent_loop_period_us);
    LOG_I(LOG_TAG_MAIN, "  zupt_accel_tol_x1000 : %u",
          (unsigned)g_active_config.ekf_zupt_accel_tol_x1000);
    LOG_I(LOG_TAG_MAIN, "  zupt_gyro_tol_x1000  : %u",
          (unsigned)g_active_config.ekf_zupt_gyro_tol_x1000);
    LOG_I(LOG_TAG_MAIN, "  zupt_min_samples     : %u",
          (unsigned)g_active_config.ekf_zupt_min_samples);
    LOG_I(LOG_TAG_MAIN, "  ai_trigger_period_ms : %u",
          (unsigned)g_active_config.ai_trigger_period_ms);
    LOG_I(LOG_TAG_MAIN, "  ai_confidence_x1000  : %u",
          (unsigned)g_active_config.ai_confidence_min_x1000);
    LOG_I(LOG_TAG_MAIN, "  ros_publish_period_ms: %u",
          (unsigned)g_active_config.ros_publish_period_ms);
    LOG_I(LOG_TAG_MAIN, "  ctrl_max_thrust_n    : %d",
          (int)g_active_config.control_max_thrust_n);
    LOG_I(LOG_TAG_MAIN, "  env_alt_max_m        : %d",
          (int)g_active_config.envelope_alt_max_m);
    LOG_I(LOG_TAG_MAIN, "  env_vel_max_ms       : %d",
          (int)g_active_config.envelope_vel_max_ms);
    LOG_I(LOG_TAG_MAIN, "=== END CONFIG ===");
}