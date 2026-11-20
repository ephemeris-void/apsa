/* ============================================================
 * HAL GPIO — IMPLEMENTATION (Zephyr)
 * ============================================================
 * IMU CS (PD14) is managed by Zephyr SPI driver (cs-gpios).
 * ============================================================ */

#include "hal_gpio.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

typedef struct {
    const struct device *port;
    gpio_pin_t           pin;
    gpio_flags_t         flags;
    bool                 initialized;
} hal_gpio_map_t;

static hal_gpio_map_t g_gpio_map[HAL_GPIO_COUNT];

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_gpio_init(void)
{
    /* ---------- IMU CS (PD14) — managed by Zephyr SPI driver ----------
     * Zephyr's cs-gpios in app.overlay handles PD14.
     * Do NOT configure it here to avoid conflict.
     */
    g_gpio_map[HAL_GPIO_IMU_CS].initialized = false;

    /* LED_STATUS — skipped */
    g_gpio_map[HAL_GPIO_LED_STATUS].initialized = false;

    /* LED_ERROR — skipped */
    g_gpio_map[HAL_GPIO_LED_ERROR].initialized = false;

    return PROJ_OK;
}

/* ---------- SET ---------- */
proj_status_t hal_gpio_set(hal_gpio_id_t id, bool level)
{
    VALIDATE_RETURN(id >= HAL_GPIO_COUNT, PROJ_ERR_INVALID);

    hal_gpio_map_t *m = &g_gpio_map[id];
    if (!m->initialized) {
        return PROJ_ERR_NOT_INIT;
    }

    if (gpio_pin_set(m->port, m->pin, (int)level) != 0) {
        return PROJ_ERR_HW;
    }
    return PROJ_OK;
}

/* ---------- GET ---------- */
proj_status_t hal_gpio_get(hal_gpio_id_t id, bool *level_out)
{
    VALIDATE_NOT_NULL(level_out);
    VALIDATE_RETURN(id >= HAL_GPIO_COUNT, PROJ_ERR_INVALID);

    hal_gpio_map_t *m = &g_gpio_map[id];
    if (!m->initialized) {
        return PROJ_ERR_NOT_INIT;
    }

    int v = gpio_pin_get(m->port, m->pin);
    if (v < 0) return PROJ_ERR_HW;

    *level_out = (v != 0);
    return PROJ_OK;
}

/* ---------- TOGGLE ---------- */
proj_status_t hal_gpio_toggle(hal_gpio_id_t id)
{
    VALIDATE_RETURN(id >= HAL_GPIO_COUNT, PROJ_ERR_INVALID);

    hal_gpio_map_t *m = &g_gpio_map[id];
    if (!m->initialized) return PROJ_ERR_NOT_INIT;

    if (gpio_pin_toggle(m->port, m->pin) != 0) {
        return PROJ_ERR_HW;
    }
    return PROJ_OK;
}