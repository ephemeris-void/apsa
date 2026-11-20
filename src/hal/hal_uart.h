/* ============================================================
 * HAL UART — ABSTRACTION LAYER
 * ============================================================
 * - Zephyr UART driver backend
 * - Blocking write (console + micro-ROS share USART3)
 * - Blocking read
 * - Thread-safe
 * - Zero dynamic allocation
 * ============================================================ */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "project_config.h"
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- UART Port IDs ---------- */
typedef enum {
    HAL_UART_TELEMETRY = 0,
    HAL_UART_ROS_AGENT = 1,
    HAL_UART_COUNT     = 2
} hal_uart_port_t;

/* ---------- Init ---------- */
proj_status_t hal_uart_init(void);

/* ---------- Blocking Write ---------- */
proj_status_t hal_uart_write(hal_uart_port_t port,
                             const uint8_t  *data,
                             size_t          len,
                             uint32_t        timeout_us);

/* ---------- Non-Blocking Write (same as blocking) ---------- */
proj_status_t hal_uart_write_async(hal_uart_port_t port,
                                   const uint8_t  *data,
                                   size_t          len);

/* ---------- Blocking Read ---------- */
proj_status_t hal_uart_read(hal_uart_port_t port,
                            uint8_t        *buf,
                            size_t          len,
                            uint32_t        timeout_us);

/* ---------- String Helpers ---------- */
proj_status_t hal_uart_puts(hal_uart_port_t port, const char *str);

/* ---------- Flush ---------- */
proj_status_t hal_uart_flush(hal_uart_port_t port, uint32_t timeout_us);

/* ---------- Stats ---------- */
uint32_t hal_uart_tx_drop_count(hal_uart_port_t port);
uint32_t hal_uart_rx_overflow_count(hal_uart_port_t port);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */