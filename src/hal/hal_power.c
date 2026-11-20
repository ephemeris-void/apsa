/* ============================================================
 * HAL POWER — IMPLEMENTATION
 * ============================================================ */

#include "hal_power.h"
#include "config/project_config.h"
#include "logger.h"
#include "error_handler.h"

/* STM32 CMSIS */
#include "stm32h7xx.h"

/* Zephyr */
#include <zephyr/kernel.h>

/* ============================================================
 * Internal State
 * ============================================================ */
static bool g_initialized = false;

/* ============================================================
 * INIT
 * ============================================================
 * Zephyr's own clock_control driver configures most of this via
 * devicetree. This function:
 *   - Verifies VOS0 is set
 *   - Enables caches explicitly
 *   - Reads actual clock from RCC
 * ============================================================ */
proj_status_t hal_power_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    /* -------- Verify VOS0 (highest performance) -------- */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0U) {
        /* Not ready — force VOS0 */
        HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
        __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

        uint32_t timeout = 1000000U;
        while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
            if (--timeout == 0U) {
                ERROR_LOG_ERROR(PROJ_ERR_HW, ERR_SRC_HAL, "VOS0 timeout");
                return PROJ_ERR_HW;
            }
        }
    }

    /* -------- Enable I-Cache + D-Cache -------- */
    SCB_EnableICache();
    SCB_EnableDCache();

    g_initialized = true;

    LOG_I(LOG_TAG_MAIN, "Power init: SYSCLK=%u Hz, HCLK=%u Hz",
          (unsigned)hal_power_sysclk_hz(),
          (unsigned)hal_power_hclk_hz());

    return PROJ_OK;
}

/* ============================================================
 * QUERY
 * ============================================================ */
uint32_t hal_power_sysclk_hz(void)
{
    return SystemCoreClock;
}

uint32_t hal_power_hclk_hz(void)
{
    return HAL_RCC_GetHCLKFreq();
}

bool hal_power_dcache_enabled(void)
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

bool hal_power_icache_enabled(void)
{
    return (SCB->CCR & SCB_CCR_IC_Msk) != 0U;
}

/* ============================================================
 * LOW-POWER: SLEEP
 * ============================================================ */
void hal_power_enter_sleep(void)
{
    __DSB();
    __WFI();
}

/* ============================================================
 * LOW-POWER: STOP
 * ============================================================
 * WARNING: Stops PLL. On wake, clocks must be reinitialized.
 * Use only in controlled paths (idle thread) with known wake sources.
 * ============================================================ */
void hal_power_enter_stop(void)
{
    /* Clear wake-up flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Enter STOP mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* On wake, clocks must be restored. Zephyr RTOS handles
     * this via its own clock control driver. */
}