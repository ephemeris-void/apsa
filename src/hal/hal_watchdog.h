/* ============================================================
 * HAL WATCHDOG — HARDWARE + SOFTWARE WATCHDOG
 * ============================================================
 * - Zephyr watchdog driver (IWDG on STM32H7)
 * - Kick with reason logging
 * - Miss detection (last kick age)
 * - Configurable timeout
 * - Safe: never kicks from ISR without explicit flag
 * - Production-grade: no silent hangs
 * ============================================================ */

#ifndef HAL_WATCHDOG_H
#define HAL_WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Init / Config
 * ============================================================ */
proj_status_t hal_watchdog_init(uint32_t timeout_ms);

/* ============================================================
 * Kick (feed)
 * ============================================================
 * Call periodically from system health thread.
 * Never call from ISR.
 * ============================================================ */
proj_status_t hal_watchdog_kick(void);

/* ============================================================
 * Feed with tag (for diagnostics)
 * ============================================================ */
proj_status_t hal_watchdog_kick_tag(const char *tag);

/* ============================================================
 * Query
 * ============================================================ */
uint32_t hal_watchdog_last_kick_ms(void);
uint32_t hal_watchdog_kick_count(void);
uint32_t hal_watchdog_timeout_ms(void);
bool     hal_watchdog_is_enabled(void);

/* ============================================================
 * Manual trigger (testing only — never in production)
 * ============================================================ */
void hal_watchdog_force_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_WATCHDOG_H */