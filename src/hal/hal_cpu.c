/* ============================================================
 * HAL CPU — IMPLEMENTATION
 * ============================================================ */

#include "hal_cpu.h"
#include "logger.h"
#include "error_handler.h"

/* Zephyr */
#include <zephyr/kernel.h>

/* STM32 CMSIS */
#include "stm32h7xx.h"

/* ============================================================
 * Internal State
 * ============================================================ */
static bool g_initialized = false;

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_cpu_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    /* Ensure trace enabled for DWT access */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    g_initialized = true;
    return PROJ_OK;
}

/* ============================================================
 * CORE QUERY
 * ============================================================ */
hal_cpu_core_t hal_cpu_get_core(void)
{
#if defined(CORE_CM7)
    return HAL_CPU_CORE_M7;
#elif defined(CORE_CM4)
    return HAL_CPU_CORE_M4;
#else
    /* Runtime detection via CPUID */
    uint32_t cpuid = SCB->CPUID;
    /* Both M7 and M4 report the same CPUID family in H755,
       but core number is compile-time. Fallback: M7 */
    (void)cpuid;
    return HAL_CPU_CORE_M7;
#endif
}

/* ============================================================
 * CRITICAL SECTION
 * ============================================================ */
hal_cpu_irq_state_t hal_cpu_irq_disable(void)
{
    uint32_t primask;
    __asm volatile ("mrs %0, primask" : "=r" (primask) :: "memory");
    __asm volatile ("cpsid i" ::: "memory");
    return primask;
}

void hal_cpu_irq_restore(hal_cpu_irq_state_t state)
{
    if ((state & 0x1U) == 0U) {
        __asm volatile ("cpsie i" ::: "memory");
    }
}

/* ============================================================
 * SYSTEM RESET
 * ============================================================ */
void hal_cpu_reset(void)
{
    LOG_W(LOG_TAG_MAIN, "CPU reset requested");

    /* Give UART time to flush */
    k_busy_wait(1000U);

    /* Trigger system reset via AIRCR */
    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                             SCB_AIRCR_SYSRESETREQ_Msk);

    /* Wait for reset */
    for (;;) {
        __asm volatile ("nop");
    }
}

/* ============================================================
 * HALT
 * ============================================================ */
void hal_cpu_halt(void)
{
    ERROR_LOG_FATAL(PROJ_ERR, ERR_SRC_HAL, "CPU halted");

    /* Disable IRQ, spin forever */
    __asm volatile ("cpsid i" ::: "memory");

    for (;;) {
        __asm volatile ("wfi");
    }
}

/* ============================================================
 * FAULT INFO
 * ============================================================ */
void hal_cpu_get_fault_info(hal_cpu_fault_info_t *info)
{
    if (info == NULL) {
        return;
    }

    info->cfsr       = SCB->CFSR;
    info->hfsr       = SCB->HFSR;
    info->dfsr       = SCB->DFSR;
    info->mmfar      = SCB->MMFAR;
    info->bfar       = SCB->BFAR;
    info->exc_return = 0U;  /* caller may fill from stack */
}