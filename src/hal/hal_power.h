/* ============================================================
 * HAL POWER — VOLTAGE / FREQUENCY / LOW-POWER MODES
 * ============================================================
 * - VOS0 (480 MHz) setup for STM32H755
 * - Cache enable (I-Cache + D-Cache)
 * - Voltage scaling
 * - Optional low-power states (sleep, stop)
 * - Production-grade: no clock drift, no brown-outs
 * ============================================================ */

#ifndef HAL_POWER_H
#define HAL_POWER_H

#include <stdint.h>
#include <stdbool.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Init
 * ============================================================
 * Sets up:
 *   - Power supply: LDO or SMPS (config-driven)
 *   - VOS0 (highest performance)
 *   - PLL 480 MHz (CPU), 240 MHz (HCLK)
 *   - I-Cache + D-Cache
 * ============================================================ */
proj_status_t hal_power_init(void);

/* ============================================================
 * Query
 * ============================================================ */
uint32_t hal_power_sysclk_hz(void);
uint32_t hal_power_hclk_hz(void);
bool     hal_power_dcache_enabled(void);
bool     hal_power_icache_enabled(void);

/* ============================================================
 * Low-Power Modes
 * ============================================================ */
void hal_power_enter_sleep(void);
void hal_power_enter_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_POWER_H */