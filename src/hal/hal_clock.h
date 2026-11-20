/* ============================================================
 * HAL CLOCK — HIGH-RESOLUTION CLOCK ACCESS
 * ============================================================
 * - DWT cycle counter (480 MHz)
 * - Microsecond timestamps
 * - Cycle-accurate profiling
 * - Monotonic, wrap-safe (32-bit diff)
 * - Production-grade: zero overhead
 * ============================================================ */

#ifndef HAL_CLOCK_H
#define HAL_CLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Init / Enable
 * ============================================================ */
proj_status_t hal_clock_init(void);

/* ============================================================
 * Cycle Counter
 * ============================================================ */
uint32_t hal_clock_cycles(void);

/* ============================================================
 * Conversions
 * ============================================================ */
uint32_t hal_clock_cycles_to_us(uint32_t cycles);
uint32_t hal_clock_us_to_cycles(uint32_t us);

/* ============================================================
 * Elapsed (wrap-safe diff)
 * ============================================================ */
static inline uint32_t hal_clock_elapsed_cycles(uint32_t start)
{
    return hal_clock_cycles() - start;
}

static inline uint32_t hal_clock_elapsed_us(uint32_t start_cycles)
{
    return hal_clock_cycles_to_us(hal_clock_cycles() - start_cycles);
}

/* ============================================================
 * Frequency Info
 * ============================================================ */
uint32_t hal_clock_cpu_hz(void);
uint32_t hal_clock_cycles_per_us(void);

/* ============================================================
 * Sanity: verify DWT is active
 * ============================================================ */
bool hal_clock_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CLOCK_H */