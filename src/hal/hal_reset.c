/* ============================================================
 * HAL RESET — IMPLEMENTATION (Zephyr)
 * ============================================================ */

#include "hal_reset.h"
#include "hal_time.h"
#include "logger.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

/* STM32 CMSIS */
#include "stm32h7xx.h"

/* ============================================================
 * Retention RAM (survives soft reset)
 * ============================================================ */
#define RESET_RETENTION_MAGIC   0xB007B007UL

typedef struct __attribute__((aligned(4))) {
    uint32_t          magic;
    uint32_t          boot_count;
    hal_reset_reason_t previous_reason;
    uint32_t          reserved[13];
} reset_retention_t;

static __attribute__((section(".noinit"))) reset_retention_t g_retention;

/* ---------- Internal state ---------- */
static hal_reset_reason_t g_current_reason = HAL_RESET_UNKNOWN;
static bool               g_initialized    = false;

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_reset_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    /* -------- Capture current reset reason -------- */
    uint32_t flags = RCC->RSR;

    if (flags & RCC_RSR_BORRSTF) {
        g_current_reason = HAL_RESET_BROWN_OUT;
    } else if (flags & RCC_RSR_IWDG1RSTF) {
        g_current_reason = HAL_RESET_WATCHDOG;
    } else if (flags & RCC_RSR_WWDG1RSTF) {
        g_current_reason = HAL_RESET_WATCHDOG;
    } else if (flags & RCC_RSR_SFT2RSTF) {
        g_current_reason = HAL_RESET_SOFTWARE;
    } else if (flags & RCC_RSR_PINRSTF) {
        g_current_reason = HAL_RESET_PIN;
    } else if (flags & RCC_RSR_PORRSTF) {
        g_current_reason = HAL_RESET_POWER_ON;
    } else {
        g_current_reason = HAL_RESET_UNKNOWN;
    }

    /* -------- Retention RAM management -------- */
    if (g_retention.magic != RESET_RETENTION_MAGIC) {
        g_retention.magic           = RESET_RETENTION_MAGIC;
        g_retention.boot_count      = 1U;
        g_retention.previous_reason = HAL_RESET_UNKNOWN;
    } else {
        g_retention.boot_count++;
        g_retention.previous_reason = g_current_reason;
    }

    /* -------- Clear RCC reset flags -------- */
    RCC->RSR |= RCC_RSR_RMVF;

    g_initialized = true;
    return PROJ_OK;
}

/* ============================================================
 * ACCESSORS
 * ============================================================ */
hal_reset_reason_t hal_reset_get_reason(void)
{
    return g_current_reason;
}

hal_reset_reason_t hal_reset_get_previous_reason(void)
{
    return g_retention.previous_reason;
}

uint32_t hal_reset_get_boot_count(void)
{
    return g_retention.boot_count;
}

const char *hal_reset_reason_name(hal_reset_reason_t reason)
{
    switch (reason) {
    case HAL_RESET_POWER_ON:      return "POWER_ON";
    case HAL_RESET_PIN:           return "PIN";
    case HAL_RESET_SOFTWARE:      return "SOFTWARE";
    case HAL_RESET_WATCHDOG:      return "WATCHDOG";
    case HAL_RESET_BROWN_OUT:     return "BROWN_OUT";
    case HAL_RESET_LOW_POWER:     return "LOW_POWER";
    case HAL_RESET_OPTION_BYTES:  return "OPTION_BYTES";
    case HAL_RESET_FIREWALL:      return "FIREWALL";
    default:                      return "UNKNOWN";
    }
}

/* ============================================================
 * SOFTWARE RESET
 * ============================================================ */
void hal_reset_software(void)
{
    LOG_W(LOG_TAG_MAIN, "Software reset requested");

    k_busy_wait(1000U);
    sys_reboot(SYS_REBOOT_COLD);
}

void hal_reset_clear_flags(void)
{
    RCC->RSR |= RCC_RSR_RMVF;
}