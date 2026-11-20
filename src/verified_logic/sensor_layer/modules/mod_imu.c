#include "../sal_core.h"
#include "../sal_types.h"
#include "imu_icm42688.h"
#include "agent/interfaces/agent_imu_if.h"
#include "common/logger.h"

#include <string.h>
#include <zephyr/kernel.h>

/* ============================================================
 * SAL MODULE — IMU (ICM-42688-P)
 * ============================================================ */

#if SAL_ENABLE_IMU

static bool g_imu_init_done = false;

static sal_status_t mod_imu_init(void)
{
    agent_status_t st = imu_init();

    if (st != AGENT_OK) {
        LOG_E(LOG_TAG_IMU, "[SAL-IMU] init failed: %d", (int)st);
        return SAL_ERR_HW;
    }

    g_imu_init_done = true;
    LOG_I(LOG_TAG_IMU, "[SAL-IMU] init OK");
    return SAL_OK;
}

static sal_status_t mod_imu_read(void *out)
{
    if (out == NULL) {
        return SAL_ERR_INVALID;
    }

    sal_imu_data_t *imu_out = (sal_imu_data_t *)out;

    agent_sensor_sample_t sample;
    agent_status_t st = imu_get_sample(&sample);

    if (st != AGENT_OK) {
        imu_out->health = SAL_HEALTH_FAIL;
        return SAL_ERR_HW;
    }

    imu_out->ax = sample.ax;
    imu_out->ay = sample.ay;
    imu_out->az = sample.az;
    imu_out->gx = sample.gx;
    imu_out->gy = sample.gy;
    imu_out->gz = sample.gz;
    imu_out->timestamp_us = sample.timestamp_us;

    switch (sample.health) {
        case AGENT_HEALTH_OK:       imu_out->health = SAL_HEALTH_OK;       break;
        case AGENT_HEALTH_DEGRADED: imu_out->health = SAL_HEALTH_DEGRADED; break;
        case AGENT_HEALTH_FAIL:     imu_out->health = SAL_HEALTH_FAIL;     break;
        default:                    imu_out->health = SAL_HEALTH_UNKNOWN;  break;
    }

    return SAL_OK;
}

static sal_status_t mod_imu_health(void)
{
    agent_health_t h = imu_health();

    switch (h) {
        case AGENT_HEALTH_OK:       return SAL_OK;
        case AGENT_HEALTH_DEGRADED: return SAL_ERR;
        case AGENT_HEALTH_FAIL:     return SAL_ERR_HW;
        default:                    return SAL_ERR_NOT_INIT;
    }
}

static void mod_imu_deinit(void)
{
    g_imu_init_done = false;
}

static const sal_sensor_t g_imu_sensor = {
    .name       = "imu",
    .bus        = SAL_BUS_SPI,
    .rate_hz    = SAL_IMU_RATE_HZ,
    .timeout_ms = SAL_IMU_TIMEOUT_MS,
    .init       = mod_imu_init,
    .read       = mod_imu_read,
    .health_fn  = mod_imu_health,
    .deinit     = mod_imu_deinit,
};

__attribute__((constructor))
static void mod_imu_auto_register(void)
{
    (void)sal_register(&g_imu_sensor);
}

#endif /* SAL_ENABLE_IMU */