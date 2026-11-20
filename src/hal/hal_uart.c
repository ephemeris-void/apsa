/* ============================================================
 * HAL UART — IMPLEMENTATION (Zephyr)
 * ============================================================
 * Console + micro-ROS share USART6 (PC6/PC7)
 * ============================================================ */

#include "hal_uart.h"
#include "logger.h"
#include "error_handler.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h> 

/* ============================================================
 * Internal State
 * ============================================================ */
typedef struct {
    const struct device *dev;
    bool                 initialized;
} hal_uart_ctx_local_t;

static hal_uart_ctx_local_t g_uart_map[HAL_UART_COUNT];

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_uart_init(void)
{
    g_uart_map[HAL_UART_TELEMETRY].dev =
        DEVICE_DT_GET(DT_NODELABEL(usart3));

    if (!device_is_ready(g_uart_map[HAL_UART_TELEMETRY].dev)) {
        return PROJ_ERR_HW;
    }
    g_uart_map[HAL_UART_TELEMETRY].initialized = true;

    g_uart_map[HAL_UART_ROS_AGENT].dev =
        DEVICE_DT_GET(DT_NODELABEL(usart3));
    g_uart_map[HAL_UART_ROS_AGENT].initialized = true;

    return PROJ_OK;
}

/* ============================================================
 * WRITE — functional
 * ============================================================ */
proj_status_t hal_uart_write(hal_uart_port_t port,
                              const uint8_t  *data,
                              size_t          len,
                              uint32_t        timeout_us)
{
    ARG_UNUSED(timeout_us);
    VALIDATE_NOT_NULL(data);
    VALIDATE_RETURN(port >= HAL_UART_COUNT, PROJ_ERR_INVALID);

    hal_uart_ctx_local_t *ctx = &g_uart_map[port];
    if (!ctx->initialized || ctx->dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    for (size_t i = 0; i < len; i++) {
        uart_poll_out(ctx->dev, data[i]);
    }
    return PROJ_OK;
}

/* ============================================================
 * WRITE ASYNC
 * ============================================================ */
proj_status_t hal_uart_write_async(hal_uart_port_t port,
                                    const uint8_t  *data,
                                    size_t          len)
{
    return hal_uart_write(port, data, len, 0);
}

/* ============================================================
 * READ (blocking)
 * ============================================================ */
proj_status_t hal_uart_read(hal_uart_port_t port,
                             uint8_t        *buf,
                             size_t          len,
                             uint32_t        timeout_us)
{
    VALIDATE_NOT_NULL(buf);
    VALIDATE_RETURN(port >= HAL_UART_COUNT, PROJ_ERR_INVALID);

    hal_uart_ctx_local_t *ctx = &g_uart_map[port];
    if (!ctx->initialized || ctx->dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    size_t count = 0;
    int64_t deadline = k_uptime_get() + (timeout_us / 1000);

    while (count < len) {
        uint8_t c;
        if (uart_poll_in(ctx->dev, &c) == 0) {
            buf[count++] = c;
        } else {
            if (k_uptime_get() >= deadline) break;
            k_usleep(50);
        }
    }
    return (count == len) ? PROJ_OK : PROJ_ERR_TIMEOUT;
}

/* ============================================================
 * PUTS
 * ============================================================ */
proj_status_t hal_uart_puts(hal_uart_port_t port, const char *str)
{
    VALIDATE_NOT_NULL(str);
    return hal_uart_write(port, (const uint8_t *)str, strlen(str), 1000);
}

/* ============================================================
 * FLUSH
 * ============================================================ */
proj_status_t hal_uart_flush(hal_uart_port_t port, uint32_t timeout_us)
{
    ARG_UNUSED(port);
    ARG_UNUSED(timeout_us);
    return PROJ_OK;
}

/* ============================================================
 * STATS
 * ============================================================ */
uint32_t hal_uart_tx_drop_count(hal_uart_port_t port)
{
    ARG_UNUSED(port);
    return 0;
}

uint32_t hal_uart_rx_overflow_count(hal_uart_port_t port)
{
    ARG_UNUSED(port);
    return 0;
}