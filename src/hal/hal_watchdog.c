/* ============================================================
 * HAL WATCHDOG — IMPLEMENTATION
 * ============================================================ */

#include "hal_watchdog.h"
#include "hal_time.h"
#include "logger.h"
#include "error_handler.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>

/* ============================================================
 * Internal State
 * ============================================================ */
static const struct device *g_wdt_dev = NULL;
static int                   g_wdt_channel = -1;
static uint32_t              g_timeout_ms = 0U;
static uint32_t              g_last_kick_ms = 0U;
static uint32_t              g_kick_count = 0U;
static bool                  g_enabled = false;

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_watchdog_init(uint32_t timeout_ms)
{
    if (g_enabled) {
        return PROJ_OK;
    }

    if (timeout_ms == 0U || timeout_ms > 60000U) {
        return PROJ_ERR_INVALID;
    }

    g_wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    if (g_wdt_dev == NULL || !device_is_ready(g_wdt_dev)) {
        ERROR_LOG_ERROR(PROJ_ERR_HW, ERR_SRC_HAL, "WDT not ready");
        return PROJ_ERR_HW;
    }

    /* Clear any pre-existing WDT (boot safety) */
    (void)wdt_disable(g_wdt_dev);

    /* Install timeout */
    struct wdt_timeout_cfg cfg = {
        .window = {
            .min = 0U,
            .max = timeout_ms,
        },
        .callback = NULL,   /* Reset on timeout — no callback */
        .flags = WDT_FLAG_RESET_SOC,
    };

    g_wdt_channel = wdt_install_timeout(g_wdt_dev, &cfg);
    if (g_wdt_channel < 0) {
        ERROR_LOG_ERROR(PROJ_ERR_HW, ERR_SRC_HAL, "WDT install failed");
        return PROJ_ERR_HW;
    }

    /* STM32H7 IWDG: WDT_OPT_PAUSE_HALTED_BY_DBG not supported → use 0 */
    if (wdt_setup(g_wdt_dev, 0) != 0) {
        ERROR_LOG_ERROR(PROJ_ERR_HW, ERR_SRC_HAL, "WDT setup failed");
        return PROJ_ERR_HW;
    }

    g_timeout_ms    = timeout_ms;
    g_last_kick_ms  = hal_time_ms_32();
    g_kick_count    = 0U;
    g_enabled       = true;

    LOG_I(LOG_TAG_MAIN, "WDT enabled, timeout=%u ms", (unsigned)timeout_ms);
    return PROJ_OK;
}

/* ============================================================
 * KICK
 * ============================================================ */
proj_status_t hal_watchdog_kick(void)
{
    if (!g_enabled || g_wdt_dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    int rc = wdt_feed(g_wdt_dev, g_wdt_channel);
    if (rc != 0) {
        return PROJ_ERR_HW;
    }

    g_last_kick_ms = hal_time_ms_32();
    g_kick_count++;

    return PROJ_OK;
}

/* ============================================================
 * KICK WITH TAG (logging optional)
 * ============================================================ */
proj_status_t hal_watchdog_kick_tag(const char *tag)
{
    proj_status_t st = hal_watchdog_kick();

    if (st != PROJ_OK && tag != NULL) {
        LOG_W(LOG_TAG_MAIN, "WDT kick failed: %s", tag);
    }

    return st;
}

/* ============================================================
 * QUERY
 * ============================================================ */
uint32_t hal_watchdog_last_kick_ms(void)
{
    return g_last_kick_ms;
}

uint32_t hal_watchdog_kick_count(void)
{
    return g_kick_count;
}

uint32_t hal_watchdog_timeout_ms(void)
{
    return g_timeout_ms;
}

bool hal_watchdog_is_enabled(void)
{
    return g_enabled;
}

/* ============================================================
 * FORCE RESET (test only)
 * ============================================================ */
void hal_watchdog_force_reset(void)
{
    LOG_W(LOG_TAG_MAIN, "WDT force reset — intentional hang");
    /* Just stop kicking and wait for hardware reset */
    while (1) {
        k_msleep(1000);
    }
}