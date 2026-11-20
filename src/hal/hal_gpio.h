/* ============================================================
 * HAL GPIO — ABSTRACTION LAYER
 * ============================================================
 * - Zephyr GPIO driver backend
 * - Pin-ID based (no direct device reference)
 * - Thread-safe
 * ============================================================ */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Pin IDs (logical) ---------- */
typedef enum {
    HAL_GPIO_IMU_CS     = 0,
    HAL_GPIO_LED_STATUS = 1,
    HAL_GPIO_LED_ERROR  = 2,
    HAL_GPIO_COUNT      = 3
} hal_gpio_id_t;

/* ---------- Init / Config ---------- */
proj_status_t hal_gpio_init(void);

/* ---------- Write ---------- */
proj_status_t hal_gpio_set(hal_gpio_id_t id, bool level);

/* ---------- Read ---------- */
proj_status_t hal_gpio_get(hal_gpio_id_t id, bool *level_out);

/* ---------- Toggle ---------- */
proj_status_t hal_gpio_toggle(hal_gpio_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */