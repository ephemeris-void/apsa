/* ============================================================
 * HAL CPU — CORE UTILITIES
 * ============================================================
 * - Critical section (interrupt-safe)
 * - Atomic memory barriers
 * - CPU ID / core detection
 * - NOP / WFE / WFI
 * - Cycle counter (DWT)
 * ============================================================ */

#ifndef HAL_CPU_H
#define HAL_CPU_H

#include <stdint.h>
#include <stdbool.h>
#include "validation.h"

/* Zephyr + CMSIS */
#include <zephyr/kernel.h>
#include <cmsis_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * CPU Core ID
 * ============================================================ */
typedef enum {
    HAL_CPU_CORE_M7      = 0,
    HAL_CPU_CORE_M4      = 1,
    HAL_CPU_CORE_UNKNOWN = 0xFF
} hal_cpu_core_t;

/* ============================================================
 * Init
 * ============================================================ */
proj_status_t hal_cpu_init(void);

/* ============================================================
 * Core Query
 * ============================================================ */
hal_cpu_core_t hal_cpu_get_core(void);

/* ============================================================
 * Critical Section
 * ============================================================ */
typedef uint32_t hal_cpu_irq_state_t;

hal_cpu_irq_state_t hal_cpu_irq_disable(void);
void                hal_cpu_irq_restore(hal_cpu_irq_state_t state);

#define HAL_CPU_CRITICAL_BEGIN()                \
    do {                                        \
        hal_cpu_irq_state_t _s = hal_cpu_irq_disable()

#define HAL_CPU_CRITICAL_END()                  \
        hal_cpu_irq_restore(_s);                \
    } while (0)

/* ============================================================
 * Memory Barriers
 * ============================================================ */
static inline void hal_cpu_dmb(void) { __asm volatile ("dmb" ::: "memory"); }
static inline void hal_cpu_dsb(void) { __asm volatile ("dsb" ::: "memory"); }
static inline void hal_cpu_isb(void) { __asm volatile ("isb" ::: "memory"); }

/* ============================================================
 * CPU Hints
 * ============================================================ */
static inline void hal_cpu_nop(void) { __asm volatile ("nop"); }
static inline void hal_cpu_wfe(void) { __asm volatile ("wfe"); }
static inline void hal_cpu_wfi(void) { __asm volatile ("wfi"); }
static inline void hal_cpu_sev(void) { __asm volatile ("sev"); }

/* ============================================================
 * System Control
 * ============================================================ */
void hal_cpu_reset(void);
void hal_cpu_halt(void);

/* ============================================================
 * Fault Info
 * ============================================================ */
typedef struct {
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t exc_return;
} hal_cpu_fault_info_t;

void hal_cpu_get_fault_info(hal_cpu_fault_info_t *info);

/* ============================================================
 * Cycle Counter (defined in hal_cpu.c)
 * ============================================================ */
uint32_t hal_cpu_cycles(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CPU_H */