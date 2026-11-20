#include "../sal_core.h"
#include "../sal_types.h"
#include "common/logger.h"

#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* ============================================================
 * SAL MODULE — MAGNETOMETER (QMC5883P) — FINAL
 * ============================================================ */

#if SAL_ENABLE_MAG

#define MAG_ADDR            0x2CU

#define QMC5883P_CHIP_ID    0x00U
#define QMC5883P_DATA_X_LSB 0x01U
#define QMC5883P_DATA_X_MSB 0x02U
#define QMC5883P_DATA_Y_LSB 0x03U
#define QMC5883P_DATA_Y_MSB 0x04U
#define QMC5883P_DATA_Z_LSB 0x05U
#define QMC5883P_DATA_Z_MSB 0x06U
#define QMC5883P_STATUS     0x09U
#define QMC5883P_CTRL_1     0x0AU
#define QMC5883P_CTRL_2     0x0BU

/* QMC5883P control values */
#define QMC5883P_MODE_SUSPEND   0x00U
#define QMC5883P_MODE_NORMAL    0x01U
#define QMC5883P_MODE_SINGLE    0x02U
#define QMC5883P_MODE_CONT      0x03U

#define QMC5883P_ODR_10HZ       0x00U
#define QMC5883P_ODR_50HZ       0x04U
#define QMC5883P_ODR_100HZ      0x08U
#define QMC5883P_ODR_200HZ      0x0CU

#define QMC5883P_OSR_512        0x00U
#define QMC5883P_OSR_256        0x40U
#define QMC5883P_OSR_128        0x80U
#define QMC5883P_OSR_64         0xC0U

#define QMC5883P_RNG_30G        0x00U
#define QMC5883P_RNG_12G        0x04U
#define QMC5883P_RNG_8G         0x08U
#define QMC5883P_RNG_2G         0x0CU

#define QMC5883P_SCALE          (100.0f / 3750.0f)   /* µT per LSB @ ±8G */

static const struct device *g_i2c_dev = NULL;
static bool                 g_mag_init_done = false;
static sal_mag_data_t       g_mag_data;

static int mag_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write(g_i2c_dev, buf, 2U, MAG_ADDR);
    return ret;
}

static int mag_read_regs(uint8_t reg, uint8_t *out, uint8_t len)
{
    return i2c_write_read(g_i2c_dev, MAG_ADDR, &reg, 1U, out, len);
}

static sal_status_t mod_mag_init(void)
{
    g_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c4));
    if (!device_is_ready(g_i2c_dev)) {
        LOG_E(LOG_TAG_MAIN, "[SAL-MAG] I2C4 not ready");
        return SAL_ERR_HW;
    }

    /* Verify CHIP_ID */
    uint8_t id = 0U;
    if (mag_read_regs(QMC5883P_CHIP_ID, &id, 1U) != 0) {
        LOG_E(LOG_TAG_MAIN, "[SAL-MAG] CHIP_ID read fail");
        return SAL_ERR_HW;
    }
    LOG_I(LOG_TAG_MAIN, "[SAL-MAG] CHIP_ID = 0x%02X", id);

    if (id != 0x80U) {
        LOG_E(LOG_TAG_MAIN, "[SAL-MAG] Wrong chip: 0x%02X", id);
        return SAL_ERR_HW;
    }

    /* Soft reset */
    int ret = mag_write_reg(QMC5883P_CTRL_2, 0x80U);
    LOG_I(LOG_TAG_MAIN, "[SAL-MAG] soft reset write ret=%d", ret);
    k_msleep(200);

    /* Set range ±8G in CTRL_2 */
    ret = mag_write_reg(QMC5883P_CTRL_2, QMC5883P_RNG_8G);
    LOG_I(LOG_TAG_MAIN, "[SAL-MAG] CTRL_2=0x%02X write ret=%d",
          QMC5883P_RNG_8G, ret);
    k_msleep(50);

    /* Set MODE = CONTINUOUS (0x03), ODR=200Hz, OSR=512 → CTRL_1 = 0x0F */
    uint8_t ctrl1_val = (QMC5883P_MODE_CONT << 0) |
                        QMC5883P_ODR_200HZ |
                        QMC5883P_OSR_512;
    /* With MODE_CONT = 0x03, ODR_200HZ = 0x0C, OSR_512 = 0x00 */
    ctrl1_val = QMC5883P_MODE_CONT | QMC5883P_ODR_200HZ | QMC5883P_OSR_512;

    ret = mag_write_reg(QMC5883P_CTRL_1, ctrl1_val);
    LOG_I(LOG_TAG_MAIN, "[SAL-MAG] CTRL_1=0x%02X write ret=%d", ctrl1_val, ret);
    k_msleep(100);

    /* Verify CTRL_1 register */
    uint8_t verify = 0U;
    if (mag_read_regs(QMC5883P_CTRL_1, &verify, 1U) == 0) {
        LOG_I(LOG_TAG_MAIN, "[SAL-MAG] CTRL_1 verify = 0x%02X (expect 0x%02X)",
              verify, ctrl1_val);
    }

    memset(&g_mag_data, 0, sizeof(g_mag_data));
    g_mag_data.health = SAL_HEALTH_UNKNOWN;
    g_mag_init_done = true;

    LOG_I(LOG_TAG_MAIN, "[SAL-MAG] init OK @ 0x%02X", MAG_ADDR);
    return SAL_OK;
}

static sal_status_t mod_mag_read(void *out)
{
    if (out == NULL || !g_mag_init_done) return SAL_ERR_NOT_INIT;

    sal_mag_data_t *mag_out = (sal_mag_data_t *)out;

    /* Read status to check DRDY */
    uint8_t status = 0U;
    if (mag_read_regs(QMC5883P_STATUS, &status, 1U) == 0) {
        static uint32_t dbg = 0;
        if (dbg < 5U) {
            LOG_I(LOG_TAG_MAIN, "[SAL-MAG] DBG status=0x%02X", status);
            dbg++;
        }
    }

    /* Burst read 6 bytes */
    uint8_t raw[6] = {0};
    if (mag_read_regs(QMC5883P_DATA_X_LSB, raw, 6U) != 0) {
        *mag_out = g_mag_data;
        return SAL_ERR;
    }

    static uint32_t dbg2 = 0;
    if (dbg2 < 5U) {
        LOG_I(LOG_TAG_MAIN, "[SAL-MAG] DBG raw=%02X %02X %02X %02X %02X %02X",
              raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
        dbg2++;
    }

    int16_t x_raw = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y_raw = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z_raw = (int16_t)((raw[5] << 8) | raw[4]);

    g_mag_data.mag_ut[0] = (float)x_raw * QMC5883P_SCALE;
    g_mag_data.mag_ut[1] = (float)y_raw * QMC5883P_SCALE;
    g_mag_data.mag_ut[2] = (float)z_raw * QMC5883P_SCALE;

    float heading = atan2f(g_mag_data.mag_ut[1], g_mag_data.mag_ut[0]) *
                    (180.0f / 3.14159265f);
    heading += SAL_MAG_DECLINATION_DEG;
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    g_mag_data.heading_deg = heading;
    g_mag_data.timestamp_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());
    g_mag_data.health = SAL_HEALTH_OK;

    *mag_out = g_mag_data;
    return SAL_OK;
}

static sal_status_t mod_mag_health(void)
{
    if (g_mag_data.health == SAL_HEALTH_OK)       return SAL_OK;
    if (g_mag_data.health == SAL_HEALTH_DEGRADED) return SAL_ERR;
    if (g_mag_data.health == SAL_HEALTH_FAIL)     return SAL_ERR_HW;
    return SAL_ERR_NOT_INIT;
}

static void mod_mag_deinit(void)
{
    if (g_i2c_dev != NULL) {
        (void)mag_write_reg(QMC5883P_CTRL_1, QMC5883P_MODE_SUSPEND);
    }
    g_mag_init_done = false;
    g_i2c_dev = NULL;
}

static const sal_sensor_t g_mag_sensor = {
    .name       = "mag",
    .bus        = SAL_BUS_I2C,
    .rate_hz    = SAL_MAG_RATE_HZ,
    .timeout_ms = SAL_MAG_TIMEOUT_MS,
    .init       = mod_mag_init,
    .read       = mod_mag_read,
    .health_fn  = mod_mag_health,
    .deinit     = mod_mag_deinit,
};

__attribute__((constructor))
static void mod_mag_auto_register(void)
{
    (void)sal_register(&g_mag_sensor);
}

#endif /* SAL_ENABLE_MAG */