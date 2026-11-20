#include "imu_icm42688.h"
#include "../agent/interfaces/agent_imu_if.h"
#include "../agent/core/agent_context.h"
#include "../agent/core/agent_config.h"
#include "../agent/trace/agent_log.h"

#include "hal/hal_spi.h"
#include "hal/hal_gpio.h"
#include "hal/hal_uart.h"
#include "hal/hal_time.h"
#include "common/logger.h"

#include <zephyr/kernel.h>
#include <stdio.h>

static bool            g_icm_initialized = false;
static agent_health_t  g_icm_health      = AGENT_HEALTH_UNKNOWN;
static uint32_t        g_icm_fail_count  = 0U;
static uint32_t        g_icm_ok_count    = 0U;

static inline void icm_delay_ms(uint32_t ms)
{
    k_busy_wait(ms * 1000U);
}

uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t val = 0xFF;
    (void)hal_spi_reg_read(HAL_SPI_BUS_IMU, reg, &val, 1000U);
    return val;
}

void icm_write_reg(uint8_t reg, uint8_t val)
{
    (void)hal_spi_reg_write(HAL_SPI_BUS_IMU, reg, val, 1000U);
}

agent_status_t icm_init(void)
{
    if (g_icm_initialized) {
        g_icm_health = AGENT_HEALTH_OK;
        return AGENT_OK;
    }

    uint8_t who = icm_read_reg(ICM_REG_WHO_AM_I);
    if (who != ICM_WHO_AM_I_VALUE) {
        g_icm_health = AGENT_HEALTH_FAIL;
        LOG_E(LOG_TAG_IMU, "[IMU] WHO_AM_I fail: 0x%02X", who);
        return AGENT_ERR_HW;
    }

    icm_write_reg(0x11, 0x01);
    icm_delay_ms(50);

    icm_write_reg(ICM_REG_ACCEL_CONFIG0, ICM_ODR_1KHZ);
    icm_delay_ms(10);

    (void)icm_read_reg(ICM_REG_GYRO_CONFIG0);

    icm_delay_ms(100);
    icm_write_reg(ICM_REG_PWR_MGMT0, 0x0F);
    icm_delay_ms(200);

    icm_delay_ms(10);
    (void)icm_read_reg(0x1F);
    (void)icm_read_reg(0x20);

    g_icm_initialized = true;
    g_icm_health      = AGENT_HEALTH_OK;
    g_icm_ok_count    = 0U;
    g_icm_fail_count  = 0U;

    LOG_I(LOG_TAG_IMU, "[IMU] init OK (WHO_AM_I=0x%02X)", who);
    return AGENT_OK;
}

agent_status_t icm_read_sample(agent_sensor_sample_t *sample)
{
    if (!g_icm_initialized) {
        if (sample != NULL) {
            sample->ax = 0; sample->ay = 0; sample->az = 0;
            sample->gx = 0; sample->gy = 0; sample->gz = 0;
            sample->timestamp_us = agent_time_now_us();
            sample->health = AGENT_HEALTH_FAIL;
        }
        return AGENT_ERR_NOT_INIT;
    }

    uint8_t buf[14] = {0};
    (void)hal_spi_burst_read(HAL_SPI_BUS_IMU, ICM_REG_ACCEL_DATA_X1, buf, 14, 1000U);

    if (sample == NULL) return AGENT_OK;

    sample->ax = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    sample->ay = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    sample->az = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    sample->gx = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    sample->gy = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    sample->gz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    sample->timestamp_us = agent_time_now_us();
    sample->health       = AGENT_HEALTH_OK;

    /* ============================================================
     * SPI sanity checks — reject any non-physical pattern.
     * These patterns indicate a floating/disconnected bus.
     * ============================================================ */

    /* 1. All-zero — MISO stuck low or sensor not powered. */
    if (sample->ax == 0 && sample->ay == 0 && sample->az == 0 &&
        sample->gx == 0 && sample->gy == 0 && sample->gz == 0) {
        g_icm_health = AGENT_HEALTH_FAIL;
        g_icm_fail_count++; g_icm_ok_count = 0U;
        agent_health_set_imu(AGENT_HEALTH_FAIL);
        return AGENT_ERR_HW;
    }

    /* 2. All-0xFF (-1) — MISO stuck high. */
    if (sample->ax == -1 && sample->ay == -1 && sample->az == -1 &&
        sample->gx == -1 && sample->gy == -1 && sample->gz == -1) {
        g_icm_health = AGENT_HEALTH_FAIL;
        g_icm_fail_count++; g_icm_ok_count = 0U;
        agent_health_set_imu(AGENT_HEALTH_FAIL);
        return AGENT_ERR_HW;
    }

    /* 3. All-0x8000 (-32768) or all-0x7FFF (32767) — floating MISO. */
    if ((sample->ax == -32768 && sample->ay == -32768 && sample->az == -32768) ||
        (sample->ax ==  32767 && sample->ay ==  32767 && sample->az ==  32767)) {
        g_icm_health = AGENT_HEALTH_FAIL;
        g_icm_fail_count++; g_icm_ok_count = 0U;
        agent_health_set_imu(AGENT_HEALTH_FAIL);
        return AGENT_ERR_HW;
    }

    /* 4. Acceleration magnitude sanity.
     *    ±16 g full scale at 2048 LSB/g → |a|_sq in [~1e6, ~1.07e9].
     *    Reject values below 0.5 g (dropped) or above 16 g (overflow). */
    {
        int32_t amag_sq = (int32_t)sample->ax * (int32_t)sample->ax +
                          (int32_t)sample->ay * (int32_t)sample->ay +
                          (int32_t)sample->az * (int32_t)sample->az;
        if (amag_sq < 1000000 || amag_sq > 1100000000) {
            g_icm_health = AGENT_HEALTH_FAIL;
            g_icm_fail_count++; g_icm_ok_count = 0U;
            agent_health_set_imu(AGENT_HEALTH_FAIL);
            return AGENT_ERR_HW;
        }
    }

    /* Auto-recovery: 10 consecutive OK samples restore health. */
    if (g_icm_health != AGENT_HEALTH_OK) {
        g_icm_ok_count++;
        if (g_icm_ok_count >= 10U) {
            g_icm_health = AGENT_HEALTH_OK;
            g_icm_ok_count = 0U; g_icm_fail_count = 0U;
            agent_health_set_imu(AGENT_HEALTH_OK);
        }
    } else {
        g_icm_ok_count = 0U;
    }

    return AGENT_OK;
}

agent_health_t icm_health(void) { return g_icm_health; }

void icm_on_failure(void)
{
    g_icm_health = AGENT_HEALTH_FAIL;
    g_icm_fail_count++; g_icm_ok_count = 0U;
    agent_health_set_imu(AGENT_HEALTH_FAIL);
}

agent_status_t imu_init(void) { return icm_init(); }
agent_status_t imu_get_sample(agent_sensor_sample_t *sample) { return icm_read_sample(sample); }
agent_health_t imu_health(void) { return icm_health(); }
void imu_on_failure(void) { icm_on_failure(); }

void icm_set_health_ok(void)
{
    g_icm_initialized = true;
    g_icm_health = AGENT_HEALTH_OK;
    g_icm_ok_count = 0U; g_icm_fail_count = 0U;
    agent_health_set_imu(AGENT_HEALTH_OK);
}