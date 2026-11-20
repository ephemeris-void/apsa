/* ============================================================
 * HAL CLOCK — IMPLEMENTATION
 * ============================================================ */

#include "hal_clock.h"
#include "config/project_config.h"

/* STM32 CMSIS */
#include "stm32h7xx.h"

/* ============================================================
 * Internal State
 * ============================================================ */
static bool     g_initialized = false;
static uint32_t g_cpu_hz = 0U;
static uint32_t g_cycles_per_us = 0U;

/* ============================================================
 * INIT — Enable DWT cycle counter
 * ============================================================ */
proj_status_t hal_clock_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    /* Enable trace and debug for DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0U;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Read CPU frequency */
    g_cpu_hz = SystemCoreClock;
    if (g_cpu_hz == 0U) {
        g_cpu_hz = CFG_CPU_FREQ_HZ;
    }

    g_cycles_per_us = g_cpu_hz / 1000000U;
    if (g_cycles_per_us == 0U) {
        g_cycles_per_us = 1U;
    }

    g_initialized = true;
    return PROJ_OK;
}

/* ============================================================
 * CYCLE COUNTER
 * ============================================================ */
uint32_t hal_clock_cycles(void)
{
    return DWT->CYCCNT;
}

/* ============================================================
 * CONVERSIONS
 * ============================================================ */
uint32_t hal_clock_cycles_to_us(uint32_t cycles)
{
    if (g_cycles_per_us == 0U) {
        return 0U;
    }
    return cycles / g_cycles_per_us;
}

uint32_t hal_clock_us_to_cycles(uint32_t us)
{
    return us * g_cycles_per_us;
}

/* ============================================================
 * FREQUENCY INFO
 * ============================================================ */
uint32_t hal_clock_cpu_hz(void)
{
    return g_cpu_hz;
}

uint32_t hal_clock_cycles_per_us(void)
{
    return g_cycles_per_us;
}

/* ============================================================
 * SANITY
 * ============================================================ */
bool hal_clock_is_running(void)
{
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}