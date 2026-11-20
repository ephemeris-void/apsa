/* ============================================================
 * HAL TIME — ABSTRACTION LAYER
 * ============================================================
 * Time API independent of RTOS.
 * Zephyr backend: k_uptime_get() / k_uptime_ticks()
 *
 * Guarantees:
 *  - Monotonic
 *  - 64-bit safe (no wrap for 500M years @ 1us)
 *  - Thread-safe (lock-free)
 *  - Zero-copy
 * ============================================================ */

#ifndef HAL_TIME_H
#define HAL_TIME_H

#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Initialization ---------- */
proj_status_t hal_time_init(void);

/* ---------- Monotonic Time (microseconds) ---------- */
uint64_t hal_time_us(void);
uint32_t hal_time_us_32(void);

/* ---------- Monotonic Time (milliseconds) ---------- */
uint64_t hal_time_ms(void);
uint32_t hal_time_ms_32(void);

/* ---------- CPU Cycle Counter (high-res) ---------- */
uint32_t hal_time_cycles(void);
uint32_t hal_time_cycles_to_us(uint32_t cycles);

/* ---------- Sleep / Delay ---------- */
void hal_sleep_us(uint32_t us);
void hal_sleep_ms(uint32_t ms);

/* ---------- Elapsed Time Helpers ---------- */
static inline uint32_t hal_elapsed_us(uint32_t start_us)
{
    return hal_time_us_32() - start_us;
}

static inline uint32_t hal_elapsed_ms(uint32_t start_ms)
{
    return hal_time_ms_32() - start_ms;
}

/* ---------- Timeout Helper ---------- */
static inline bool hal_timeout_elapsed_us(uint32_t start_us, uint32_t timeout_us)
{
    return ((hal_time_us_32() - start_us) >= timeout_us);
}

static inline bool hal_timeout_elapsed_ms(uint32_t start_ms, uint32_t timeout_ms)
{
    return ((hal_time_ms_32() - start_ms) >= timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_TIME_H */