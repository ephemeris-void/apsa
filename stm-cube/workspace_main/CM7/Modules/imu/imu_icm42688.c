#include "imu_icm42688.h"
#include "../agent/interfaces/agent_imu_if.h"
#include "../agent/core/agent_context.h"
#include "../agent/core/agent_config.h"
#include "../agent/trace/agent_log.h"

#include "main.h"
#include <stdio.h>

/* ============================================================
 * ICM-42688-P IMU DRIVER — IMPLEMENTATION
 * ============================================================
 * Board:  NUCLEO-H755ZI-Q
 * Bus:    SPI1 @ 3.75 MHz (prescaler 64)
 * Pins:   PA5=SCK, PA6=MISO, PB5=MOSI, PD14=CS
 * ODR:    1 kHz
 * Range:  ±16g, ±2000 dps
 * WCET:   50 µs per sample
 * ============================================================ */

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;

/* ---------- Debug globals (defined in main.c) ---------- */
extern int16_t dbg_ax;
extern int16_t dbg_ay;
extern int16_t dbg_az;
extern int16_t dbg_gx;
extern int16_t dbg_gy;
extern int16_t dbg_gz;

static bool            g_icm_initialized = false;
static agent_health_t  g_icm_health      = AGENT_HEALTH_UNKNOWN;
static uint32_t        g_icm_fail_count  = 0U;
static uint32_t        g_icm_ok_count    = 0U;

/* ---------- CS Control ---------- */
#define CS_LOW()   (GPIOD->BSRR = (uint32_t)GPIO_PIN_14 << 16U)
#define CS_HIGH()  (GPIOD->BSRR = (uint32_t)GPIO_PIN_14)

/* ---------- SPI Transfer (single byte via HAL) ---------- */
AGENT_INLINE_FAST uint8_t spi_xfer(uint8_t data)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

/* ---------- Low-Level Register Access ---------- */
uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80U), 0x00U};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    return rx[1];
}

void icm_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), val};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

/* ---------- INIT ---------- */
agent_status_t icm_init(void)
{
    char buf[128];
    int len;

    if (g_icm_initialized) {
        g_icm_health = AGENT_HEALTH_OK;
        return AGENT_OK;
    }

    /* ---------- STEP 1: WHO_AM_I check ---------- */
    uint8_t who = icm_read_reg(ICM_REG_WHO_AM_I);

    len = sprintf(buf, "[ICM_INIT] WHO_AM_I = 0x%02X (expect 0x47)\r\n", who);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);

    if (who != ICM_WHO_AM_I_VALUE) {
        len = sprintf(buf, "[ICM_INIT] WHO_AM_I FAIL — aborting\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);
        agent_log_event(AGENT_LOG_IMU_FAIL, who, 0U);
        g_icm_health = AGENT_HEALTH_FAIL;
        return AGENT_ERR_HW;
    }

    /* ---------- STEP 2: Power Management ---------- */
    icm_write_reg(ICM_REG_PWR_MGMT0, ICM_PWR_MGMT0_LN);
    HAL_Delay(50U);

    uint8_t pwr_check = icm_read_reg(ICM_REG_PWR_MGMT0);
    len = sprintf(buf, "[ICM_INIT] PWR_MGMT0 wrote=0x%02X read=0x%02X\r\n",
                  ICM_PWR_MGMT0_LN, pwr_check);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);

    /* ---------- STEP 3: ACCEL Config ---------- */
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, ICM_ODR_1KHZ);
    HAL_Delay(10U);

    uint8_t acc_check = icm_read_reg(ICM_REG_ACCEL_CONFIG0);
    len = sprintf(buf, "[ICM_INIT] ACCEL_CFG0 wrote=0x%02X read=0x%02X\r\n",
                  ICM_ODR_1KHZ, acc_check);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);

    /* ---------- STEP 4: GYRO Config ---------- */
    icm_write_reg(ICM_REG_GYRO_CONFIG0, ICM_ODR_1KHZ);
    HAL_Delay(10U);

    uint8_t gyr_check = icm_read_reg(ICM_REG_GYRO_CONFIG0);
    len = sprintf(buf, "[ICM_INIT] GYRO_CFG0 wrote=0x%02X read=0x%02X\r\n",
                  ICM_ODR_1KHZ, gyr_check);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);

    /* ---------- STEP 5: First sample read (test) ---------- */
    HAL_Delay(10U);

    uint8_t test_ax_h = icm_read_reg(0x1F);
    uint8_t test_ax_l = icm_read_reg(0x20);
    uint8_t test_ay_h = icm_read_reg(0x21);
    uint8_t test_ay_l = icm_read_reg(0x22);
    uint8_t test_az_h = icm_read_reg(0x23);
    uint8_t test_az_l = icm_read_reg(0x24);

    int16_t test_ax = (int16_t)(((uint16_t)test_ax_h << 8) | test_ax_l);
    int16_t test_ay = (int16_t)(((uint16_t)test_ay_h << 8) | test_ay_l);
    int16_t test_az = (int16_t)(((uint16_t)test_az_h << 8) | test_az_l);

    len = sprintf(buf, "[ICM_INIT] First sample: ax=%d ay=%d az=%d\r\n",
                  test_ax, test_ay, test_az);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);

    /* ---------- STEP 6: Mark initialized ---------- */
    g_icm_initialized = true;
    g_icm_health      = AGENT_HEALTH_OK;
    g_icm_ok_count    = 0U;
    g_icm_fail_count  = 0U;

    len = sprintf(buf, "[ICM_INIT] Initialization COMPLETE\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);

    return AGENT_OK;
}

/* ---------- READ SAMPLE ---------- */
agent_status_t icm_read_sample(agent_sensor_sample_t *sample)
{
    if (!g_icm_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    /* Burst read: 1 address byte + 14 data bytes (single SPI transaction) */
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};

    tx[0] = (uint8_t)(ICM_REG_ACCEL_DATA_X1 | 0x80U);

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 15, 100);
    CS_HIGH();

    /* rx[0] = address-phase dummy, rx[1..14] = data */
    uint8_t *buf = &rx[1];

    if (sample == NULL) {
        return AGENT_OK;
    }

    sample->ax = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    sample->ay = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    sample->az = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    sample->gx = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    sample->gy = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    sample->gz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    sample->timestamp_us = agent_time_now_us();
    sample->health       = AGENT_HEALTH_OK;

    /* ---------- Update debug globals for main.c display ---------- */
    dbg_ax = sample->ax;
    dbg_ay = sample->ay;
    dbg_az = sample->az;
    dbg_gx = sample->gx;
    dbg_gy = sample->gy;
    dbg_gz = sample->gz;

    /* ---------- Runtime IMU failure detection ----------
     * Check 1: Range validation (soft fail → DEGRADED)
     */
    if (sample->ax < -32000 || sample->ax > 32000 ||
        sample->ay < -32000 || sample->ay > 32000 ||
        sample->az < -32000 || sample->az > 32000 ||
        sample->gx < -32000 || sample->gx > 32000 ||
        sample->gy < -32000 || sample->gy > 32000 ||
        sample->gz < -32000 || sample->gz > 32000) {

        g_icm_health = AGENT_HEALTH_DEGRADED;
        g_icm_fail_count++;
        g_icm_ok_count = 0U;
        return AGENT_ERR;
    }

    /* Check 2: All-zero pattern (hard fail → VIRTUALIZE) */
    if (sample->ax == 0 && sample->ay == 0 && sample->az == 0 &&
        sample->gx == 0 && sample->gy == 0 && sample->gz == 0) {

        g_icm_health = AGENT_HEALTH_FAIL;
        g_icm_fail_count++;
        g_icm_ok_count = 0U;
        agent_log_event(AGENT_LOG_IMU_FAIL, g_icm_fail_count, 0U);
        return AGENT_ERR_HW;
    }

    /* ---------- RUNTIME RECOVERY ----------
     * If we were FAIL or DEGRADED, but now receiving good data
     * for N consecutive samples → transition to OK.
     */
    if (g_icm_health != AGENT_HEALTH_OK) {
        g_icm_ok_count++;

        if (g_icm_ok_count >= 10U) {         /* 10 samples = 10 ms @ 1 kHz */
            g_icm_health     = AGENT_HEALTH_OK;
            g_icm_ok_count   = 0U;
            g_icm_fail_count = 0U;
            agent_health_set_imu(AGENT_HEALTH_OK);   /* Sync agent context */
            agent_log_event(AGENT_LOG_SENSOR_RECOVERED, 0U, 0U);
        }
    } else {
        g_icm_ok_count = 0U;
    }

    return AGENT_OK;
}

/* ---------- HEALTH ---------- */
agent_health_t icm_health(void)
{
    return g_icm_health;
}

/* ---------- FAILURE INJECTION ---------- */
void icm_on_failure(void)
{
    g_icm_health     = AGENT_HEALTH_FAIL;
    g_icm_fail_count++;
    g_icm_ok_count   = 0U;
    agent_health_set_imu(AGENT_HEALTH_FAIL);   /* Sync agent context */
    agent_log_event(AGENT_LOG_IMU_FAIL, g_icm_fail_count, 0U);
}

/* ============================================================
 * INTERFACE GLUE (agent_imu_if.h)
 * ============================================================ */

agent_status_t imu_init(void)
{
    return icm_init();
}

agent_status_t imu_get_sample(agent_sensor_sample_t *sample)
{
    return icm_read_sample(sample);
}

agent_health_t imu_health(void)
{
    return icm_health();
}

void imu_on_failure(void)
{
    icm_on_failure();
}

/* ---------- FORCE HEALTH OK (used when main.c does init) ---------- */
void icm_set_health_ok(void)
{
    g_icm_initialized = true;
    g_icm_health      = AGENT_HEALTH_OK;
    g_icm_ok_count    = 0U;
    g_icm_fail_count  = 0U;
    agent_health_set_imu(AGENT_HEALTH_OK);   /* Sync agent context */
}