/* ============================================================
 * HAL RESET — RESET CAUSE + SYSTEM RESET
 * ============================================================
 * - Read reset cause from RCC
 * - Persist reset reason to retention RAM
 * - Software reset with safe shutdown hooks
 * - Distinguish: power-on, watchdog, software, brown-out, pin
 * - Production-grade diagnostics
 * ============================================================ */

#ifndef HAL_RESET_H
#define HAL_RESET_H

#include <stdint.h>
#include <stdbool.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Reset Reason Codes
 * ============================================================ */
typedef enum {
    HAL_RESET_UNKNOWN        = 0,
    HAL_RESET_POWER_ON       = 1,
    HAL_RESET_PIN            = 2,
    HAL_RESET_SOFTWARE       = 3,
    HAL_RESET_WATCHDOG       = 4,
    HAL_RESET_BROWN_OUT      = 5,
    HAL_RESET_LOW_POWER      = 6,
    HAL_RESET_OPTION_BYTES   = 7,
    HAL_RESET_FIREWALL       = 8
} hal_reset_reason_t;

/* ============================================================
 * API
 * ============================================================ */

/* Init: capture current reset reason */
proj_status_t hal_reset_init(void);

/* Get reason for THIS boot */
hal_reset_reason_t hal_reset_get_reason(void);

/* Get human-readable name */
const char *hal_reset_reason_name(hal_reset_reason_t reason);

/* Get boot count (persisted in retention RAM) */
uint32_t hal_reset_get_boot_count(void);

/* Get previous reset reason (from retention RAM) */
hal_reset_reason_t hal_reset_get_previous_reason(void);

/* Request a software reset (system will reboot) */
void hal_reset_software(void);

/* Clear any latched reset flags (call AFTER reading) */
void hal_reset_clear_flags(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_RESET_H */