#include "sal_core.h"
#include "common/logger.h"

#include <string.h>
#include <zephyr/kernel.h>

#define SAL_MAX_SENSORS     8U
#define SAL_DBG_ITERS       2U

static sal_sensor_t  g_registry[SAL_MAX_SENSORS];
static uint32_t      g_registered = 0U;

static sal_imu_data_t   g_imu_data;
static sal_gps_data_t   g_gps_data;
static sal_mag_data_t   g_mag_data;
static sal_baro_data_t  g_baro_data;
static sal_lora_data_t  g_lora_data;

static sal_stats_t g_stats;

sal_status_t sal_register(const sal_sensor_t *sensor)
{
    if (sensor == NULL || sensor->name == NULL) {
        return SAL_ERR_INVALID;
    }

    if (g_registered >= SAL_MAX_SENSORS) {
        LOG_E(LOG_TAG_MAIN, "[SAL] registry full");
        return SAL_ERR;
    }

    g_registry[g_registered] = *sensor;
    g_registry[g_registered].health         = SAL_HEALTH_UNKNOWN;
    g_registry[g_registered].last_update_us = 0U;
    g_registry[g_registered].read_count     = 0U;
    g_registry[g_registered].fail_count     = 0U;
    g_registry[g_registered].registered     = true;

    g_registered++;
    g_stats.registered_count++;

    return SAL_OK;
}

sal_status_t sal_init_all(void)
{
    sal_status_t result = SAL_OK;
    uint32_t now_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());

    LOG_I(LOG_TAG_MAIN, "[SAL] init_all: %u sensors", (unsigned)g_registered);

    for (uint32_t i = 0U; i < g_registered; i++) {
        sal_sensor_t *s = &g_registry[i];

        if (!s->registered || s->init == NULL) {
            continue;
        }

        sal_status_t st = s->init();

        if (st == SAL_OK) {
            s->health = SAL_HEALTH_OK;
            s->last_update_us = now_us;
            g_stats.init_ok_count++;
            LOG_I(LOG_TAG_MAIN, "[SAL] %s init OK", s->name);
        } else {
            s->health = SAL_HEALTH_FAIL;
            g_stats.init_fail_count++;
            result = SAL_ERR;
            LOG_E(LOG_TAG_MAIN, "[SAL] %s init FAIL (st=%d)", s->name, (int)st);
        }
    }

    return result;
}

sal_status_t sal_read_all(void)
{
    sal_status_t result = SAL_OK;
    uint32_t now_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());

    static uint32_t dbg_cnt[SAL_MAX_SENSORS] = {0};

    for (uint32_t i = 0U; i < g_registered; i++) {
        sal_sensor_t *s = &g_registry[i];

        if (!s->registered || s->read == NULL) {
            continue;
        }

        if (s->rate_hz > 0U) {
            uint32_t period_us = 1000000U / s->rate_hz;
            uint32_t elapsed = now_us - s->last_update_us;

            if (elapsed < period_us) {
                continue;
            }
        }

        void *out = NULL;
        if      (strcmp(s->name, "imu")  == 0) out = &g_imu_data;
        else if (strcmp(s->name, "gps")  == 0) out = &g_gps_data;
        else if (strcmp(s->name, "mag")  == 0) out = &g_mag_data;
        else if (strcmp(s->name, "baro") == 0) out = &g_baro_data;
        else if (strcmp(s->name, "lora") == 0) out = &g_lora_data;

        if (out == NULL) continue;

        bool dbg = (dbg_cnt[i] < SAL_DBG_ITERS);
        if (dbg) LOG_I(LOG_TAG_MAIN, "[SAL] BEFORE %s", s->name);

        sal_status_t st = s->read(out);

        if (dbg) {
            LOG_I(LOG_TAG_MAIN, "[SAL] AFTER  %s st=%d", s->name, (int)st);
            dbg_cnt[i]++;
        }

        s->last_update_us = now_us;

        if (st == SAL_OK) {
            s->read_count++;
            s->health = SAL_HEALTH_OK;
            g_stats.read_ok_count++;
        } else if (st == SAL_ERR) {
            s->fail_count++;
            g_stats.read_fail_count++;
        } else {
            s->fail_count++;
            g_stats.read_fail_count++;

            uint32_t age_ms = (now_us - s->last_update_us) / 1000U;

            if (age_ms > s->timeout_ms) {
                if (s->health != SAL_HEALTH_FAIL) {
                    LOG_E(LOG_TAG_MAIN, "[SAL] %s TIMEOUT (%u ms)",
                          s->name, (unsigned)age_ms);
                }
                s->health = SAL_HEALTH_FAIL;
            } else {
                s->health = SAL_HEALTH_DEGRADED;
            }

            result = SAL_ERR;
        }
    }

    return result;
}

sal_status_t sal_health_all(void)
{
    for (uint32_t i = 0U; i < g_registered; i++) {
        sal_sensor_t *s = &g_registry[i];
        if (!s->registered || s->health_fn == NULL) continue;
        (void)s->health_fn();
    }
    return SAL_OK;
}

void sal_deinit_all(void)
{
    for (uint32_t i = 0U; i < g_registered; i++) {
        sal_sensor_t *s = &g_registry[i];
        if (s->registered && s->deinit != NULL) {
            s->deinit();
        }
    }
}

sal_status_t sal_get_imu(sal_imu_data_t *out)
{ if (out == NULL) return SAL_ERR_INVALID; *out = g_imu_data; return SAL_OK; }

sal_status_t sal_get_gps(sal_gps_data_t *out)
{ if (out == NULL) return SAL_ERR_INVALID; *out = g_gps_data; return SAL_OK; }

sal_status_t sal_get_mag(sal_mag_data_t *out)
{ if (out == NULL) return SAL_ERR_INVALID; *out = g_mag_data; return SAL_OK; }

sal_status_t sal_get_baro(sal_baro_data_t *out)
{ if (out == NULL) return SAL_ERR_INVALID; *out = g_baro_data; return SAL_OK; }

sal_status_t sal_get_lora(sal_lora_data_t *out)
{ if (out == NULL) return SAL_ERR_INVALID; *out = g_lora_data; return SAL_OK; }

sal_health_t sal_get_health(const char *name)
{
    if (name == NULL) return SAL_HEALTH_UNKNOWN;
    for (uint32_t i = 0U; i < g_registered; i++) {
        if (strcmp(g_registry[i].name, name) == 0) {
            return g_registry[i].health;
        }
    }
    return SAL_HEALTH_UNKNOWN;
}

extern sal_status_t sal_lora_send_raw(const uint8_t *data, uint16_t len);

sal_status_t sal_lora_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) return SAL_ERR_INVALID;
#if SAL_ENABLE_LORA
    return sal_lora_send_raw(data, len);
#else
    (void)data; (void)len;
    return SAL_ERR_NOT_INIT;
#endif
}

const sal_stats_t* sal_get_stats(void)
{
    return &g_stats;
}

void sal_dump_registry(void)
{
    LOG_I(LOG_TAG_MAIN, "[SAL] === Registry ===");
    for (uint32_t i = 0U; i < g_registered; i++) {
        sal_sensor_t *s = &g_registry[i];
        LOG_I(LOG_TAG_MAIN,
              "[SAL] %s bus=%d rate=%u Hz health=%d read=%u fail=%u",
              s->name, (int)s->bus, (unsigned)s->rate_hz,
              (int)s->health, (unsigned)s->read_count, (unsigned)s->fail_count);
    }
    LOG_I(LOG_TAG_MAIN, "[SAL] Registered: %u", (unsigned)g_registered);
}