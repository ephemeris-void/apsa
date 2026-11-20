/* ============================================================
 * HAL TIME — IMPLEMENTATION
 * ============================================================ */

#include "hal_time.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/atomic.h>

/* ARM CMSIS */
#include "stm32h7xx.h"

/* ---------- Internal State ---------- */
static atomic_t g_initialized = ATOMIC_INIT(0);
static uint32_t g_cycles_per_us = 0U;

/* ---------- INIT ---------- */
proj_status_t hal_time_init(void)
{
    if (atomic_get(&g_initialized)) {
        return PROJ_OK;
    }

    g_cycles_per_us = CFG_CPU_FREQ_HZ / 1000000U;
    if (g_cycles_per_us == 0U) {
        g_cycles_per_us = 1U;
    }

    /* Enable DWT cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    atomic_set(&g_initialized, 1);
    return PROJ_OK;
}

/* ---------- MONOTONIC US ---------- */
uint64_t hal_time_us(void)
{
    return (uint64_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

uint32_t hal_time_us_32(void)
{
    return (uint32_t)hal_time_us();
}

/* ---------- MONOTONIC MS ---------- */
uint64_t hal_time_ms(void)
{
    return (uint64_t)k_uptime_get();
}

uint32_t hal_time_ms_32(void)
{
    return (uint32_t)k_uptime_get_32();
}

/* ---------- CPU CYCLES ---------- */
uint32_t hal_time_cycles(void)
{
    return DWT->CYCCNT;
}

uint32_t hal_time_cycles_to_us(uint32_t cycles)
{
    if (g_cycles_per_us == 0U) {
        return 0U;
    }
    return cycles / g_cycles_per_us;
}

/* ---------- SLEEP ---------- */
void hal_sleep_us(uint32_t us)
{
    if (us == 0U) {
        return;
    }
    k_busy_wait(us);
}

void hal_sleep_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    k_msleep((int32_t)ms);
}