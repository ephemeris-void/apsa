#include "../sal_core.h"
#include "../sal_types.h"
#include "common/logger.h"

#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* ============================================================
 * SAL MODULE — BAROMETER (DPS310) — FINAL v17
 * ============================================================
 * Aligned with verified bare-metal working code:
 *   ✅ 18-byte burst read from 0x10 (correct coeff layout)
 *   ✅ DPS_KP = 7864320.0f for 8x oversample
 *   ✅ PRS_CFG = 0x23, TMP_CFG = 0x23 (8x oversample)
 *   ✅ CFG_REG = 0x00
 *   ✅ Temperature errata workaround (0x0E/0x0F/0x62)
 *   ✅ sign_ext helper exactly as working code
 * ============================================================ */

#if SAL_ENABLE_BARO

#define DPS_REG_PSR_B2    0x00U
#define DPS_REG_TMP_B2    0x03U
#define DPS_REG_PRS_CFG   0x06U
#define DPS_REG_TMP_CFG   0x07U
#define DPS_REG_MEAS_CFG  0x08U
#define DPS_REG_CFG_REG   0x09U
#define DPS_REG_RESET     0x0CU
#define DPS_REG_ID        0x0DU
#define DPS_REG_COEF      0x10U
#define DPS_REG_COEF_SRCE 0x28U

#define DPS_PRODUCT_ID    0x10U
#define DPS_KP            7864320.0f
#define DPS_KT            7864320.0f

#define DPS_MEAS_CFG_CONT_BOTH  0x07U
#define DPS_RESET_SOFTRESET     0x89U

static const struct device *g_i2c_dev = NULL;
static bool                 g_baro_init_done = false;
static sal_baro_data_t      g_baro_data;
static uint16_t             g_dps_addr = SAL_BARO_I2C_ADDR;

static struct {
    int32_t c0, c1, c00, c10;
    int32_t c01, c11, c20, c21, c30;
} cf;

/* ============================================================
 * I2C helpers
 * ============================================================ */
static int dps_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_write(g_i2c_dev, buf, 2U, g_dps_addr);
}

static int dps_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2c_write_read(g_i2c_dev, g_dps_addr, &reg, 1U, buf, len);
}

/* Exact sign_ext from working code */
static int32_t sign_ext(uint32_t v, uint8_t bits)
{
    if (v & (1UL << (bits - 1))) v |= ~((1UL << bits) - 1);
    return (int32_t)v;
}

/* ============================================================
 * dps310_init()
 * ============================================================
 * Exact logic from verified working bare-metal code.
 * ============================================================ */
static int dps310_init(void)
{
    uint8_t id = 0, r = 0, c[18];

    g_dps_addr = (0x77);
    if (dps_read(DPS_REG_ID, &id, 1) != 0) {
        g_dps_addr = (0x76);
        if (dps_read(DPS_REG_ID, &id, 1) != 0) {
            LOG_E(LOG_TAG_MAIN, "[SAL-BARO] ID read fail");
            return -1;
        }
    }
    if (id != DPS_PRODUCT_ID) {
        LOG_E(LOG_TAG_MAIN, "[SAL-BARO] wrong ID 0x%02X", id);
        return -2;
    }
    LOG_I(LOG_TAG_MAIN, "[SAL-BARO] ID=0x%02X addr=0x%02X", id, g_dps_addr);

    dps_write(DPS_REG_RESET, DPS_RESET_SOFTRESET);
    k_msleep(50);

    /* ---- Temperature errata workaround (manufacturer) ---- */
    uint8_t wa = 0;
    dps_read(0x32, &wa, 1);
    if ((wa & 0x02) == 0) {
        LOG_I(LOG_TAG_MAIN, "[SAL-BARO] applying temp workaround (0x32=0x%02X)", wa);
        dps_write(0x0E, 0xA5);
        dps_write(0x0F, 0x96);
        dps_write(0x62, 0x02);
        dps_write(0x0E, 0x00);
        dps_write(0x0F, 0x00);
    } else {
        LOG_I(LOG_TAG_MAIN, "[SAL-BARO] temp workaround NOT needed");
    }

    /* Wait COEF_RDY + SENSOR_RDY — 500ms timeout */
    uint32_t t0 = k_uptime_get_32();
    do {
        dps_read(DPS_REG_MEAS_CFG, &r, 1);
        if ((k_uptime_get_32() - t0) > 500U) {
            LOG_E(LOG_TAG_MAIN, "[SAL-BARO] COEF_RDY timeout (0x08=0x%02X)", r);
            return -3;
        }
    } while ((r & 0xC0) != 0xC0);
    LOG_I(LOG_TAG_MAIN, "[SAL-BARO] COEF_RDY ok (0x08=0x%02X)", r);

    /* Read 18 coefficients from 0x10 (burst) */
    if (dps_read(DPS_REG_COEF, c, 18) != 0) {
        LOG_E(LOG_TAG_MAIN, "[SAL-BARO] coef read fail");
        return -4;
    }

    cf.c0  = sign_ext(((uint32_t)c[0] << 4) | (c[1] >> 4), 12);
    cf.c1  = sign_ext((((uint32_t)c[1] & 0x0F) << 8) | c[2], 12);
    cf.c00 = sign_ext(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4), 20);
    cf.c10 = sign_ext((((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7], 20);
    cf.c01 = sign_ext(((uint32_t)c[8]  << 8) | c[9],  16);
    cf.c11 = sign_ext(((uint32_t)c[10] << 8) | c[11], 16);
    cf.c20 = sign_ext(((uint32_t)c[12] << 8) | c[13], 16);
    cf.c21 = sign_ext(((uint32_t)c[14] << 8) | c[15], 16);
    cf.c30 = sign_ext(((uint32_t)c[16] << 8) | c[17], 16);

    LOG_I(LOG_TAG_MAIN, "[SAL-BARO] CF c0=%d c1=%d c00=%d c10=%d",
          (int)cf.c0, (int)cf.c1, (int)cf.c00, (int)cf.c10);

    uint8_t src = 0;
    dps_read(DPS_REG_COEF_SRCE, &src, 1);
    uint8_t tmp_ext = src & 0x80;

    dps_write(DPS_REG_PRS_CFG, 0x23);       /* 8x oversample, 16Hz */
    dps_write(DPS_REG_TMP_CFG, tmp_ext | 0x23);
    dps_write(DPS_REG_CFG_REG, 0x00);
    dps_write(DPS_REG_MEAS_CFG, DPS_MEAS_CFG_CONT_BOTH);

    k_msleep(300);
    return 0;
}

/* ============================================================
 * dps310_read()
 * ============================================================ */
static int dps310_read(float *temp_c, float *press_pa)
{
    uint8_t b[6];
    if (dps_read(DPS_REG_PSR_B2, b, 6) != 0) return -1;

    int32_t praw = sign_ext(((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2], 24);
    int32_t traw = sign_ext(((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5], 24);

    float tsc = (float)traw / DPS_KT;
    float psc = (float)praw / DPS_KP;

    *temp_c = (float)cf.c0 * 0.5f + (float)cf.c1 * tsc;
    *press_pa = (float)cf.c00
              + psc * ((float)cf.c10 + psc * ((float)cf.c20 + psc * (float)cf.c30))
              + tsc * (float)cf.c01
              + tsc * psc * ((float)cf.c11 + psc * (float)cf.c21);
    return 0;
}

/* ============================================================
 * SAL init
 * ============================================================ */
static sal_status_t mod_baro_init(void)
{
    g_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(g_i2c_dev)) {
        LOG_E(LOG_TAG_MAIN, "[SAL-BARO] I2C1 not ready");
        return SAL_ERR_HW;
    }

    int ready = -1;
    for (int attempt = 0; attempt < 5; attempt++) {
        ready = dps310_init();
        if (ready == 0) break;
        LOG_W(LOG_TAG_MAIN, "[SAL-BARO] init retry %d (err %d)", attempt, ready);
        k_msleep(200);
    }
    if (ready != 0) {
        LOG_E(LOG_TAG_MAIN, "[SAL-BARO] init FAIL (err %d)", ready);
        g_baro_data.health = SAL_HEALTH_FAIL;
        return SAL_ERR_HW;
    }

    memset(&g_baro_data, 0, sizeof(g_baro_data));
    g_baro_data.health = SAL_HEALTH_UNKNOWN;
    g_baro_init_done = true;

    LOG_I(LOG_TAG_MAIN, "[SAL-BARO] init OK");
    return SAL_OK;
}

/* ============================================================
 * SAL read
 * ============================================================ */
static sal_status_t mod_baro_read(void *out)
{
    if (out == NULL || !g_baro_init_done) return SAL_ERR_NOT_INIT;

    sal_baro_data_t *baro_out = (sal_baro_data_t *)out;

    float t = 0.0f, p = 0.0f;
    if (dps310_read(&t, &p) != 0) {
        *baro_out = g_baro_data;
        return SAL_ERR;
    }

    /* Sanity: 300-1100 hPa, -40..85 C */
    if (p < 30000.0f || p > 110000.0f) {
        static uint32_t dbg = 0;
        if (dbg < 5U) {
            LOG_W(LOG_TAG_MAIN, "[SAL-BARO] P out of range: %.2f Pa", (double)p);
            dbg++;
        }
        *baro_out = g_baro_data;
        return SAL_ERR;
    }

    float alt_m = 44330.0f * (1.0f - powf(p / SAL_BARO_SEA_LEVEL_PA, 0.1903f));

    g_baro_data.pressure_pa   = p;
    g_baro_data.temperature_c = t;
    g_baro_data.altitude_m    = alt_m;
    g_baro_data.timestamp_us  = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());
    g_baro_data.health        = SAL_HEALTH_OK;

    *baro_out = g_baro_data;
    return SAL_OK;
}

static sal_status_t mod_baro_health(void)
{
    if (g_baro_data.health == SAL_HEALTH_OK)       return SAL_OK;
    if (g_baro_data.health == SAL_HEALTH_DEGRADED) return SAL_ERR;
    if (g_baro_data.health == SAL_HEALTH_FAIL)     return SAL_ERR_HW;
    return SAL_ERR_NOT_INIT;
}

static void mod_baro_deinit(void)
{
    if (g_i2c_dev != NULL) {
        (void)dps_write(DPS_REG_MEAS_CFG, 0x00U);
    }
    g_baro_init_done = false;
    g_i2c_dev = NULL;
}

static const sal_sensor_t g_baro_sensor = {
    .name       = "baro",
    .bus        = SAL_BUS_I2C,
    .rate_hz    = SAL_BARO_RATE_HZ,
    .timeout_ms = SAL_BARO_TIMEOUT_MS,
    .init       = mod_baro_init,
    .read       = mod_baro_read,
    .health_fn  = mod_baro_health,
    .deinit     = mod_baro_deinit,
};

__attribute__((constructor))
static void mod_baro_auto_register(void)
{
    (void)sal_register(&g_baro_sensor);
}

#endif /* SAL_ENABLE_BARO */