#include <uxr/client/transport.h>

#include <microros_transports.h>
#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3,1,0)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#else
#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#endif

#include <string.h>
#include <stdbool.h>

/* ============================================================
 * micro-ROS transport on USART6 (PC6/PC7)
 * ============================================================ */
#define UART_NODE DT_NODELABEL(usart6)

#define RING_BUF_SIZE 2048

static char uart_in_buffer[RING_BUF_SIZE];
static char uart_out_buffer[RING_BUF_SIZE];

static struct ring_buf out_ringbuf;
static struct ring_buf in_ringbuf;

/* ============================================================
 * UART RX ISR — ring buffer fill
 * ============================================================ */
static void uart_fifo_callback(const struct device *dev, void *args)
{
    (void)args;

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            int recv_len;
            char buffer[64];
            size_t len = MIN(ring_buf_space_get(&in_ringbuf), sizeof(buffer));

            if (len > 0) {
                recv_len = uart_fifo_read(dev, buffer, len);
                if (recv_len > 0) {
                    ring_buf_put(&in_ringbuf, buffer, recv_len);
                }
            }
        }
    }
}

/* ============================================================
 * TRANSPORT OPEN
 * ============================================================ */
bool zephyr_transport_open(struct uxrCustomTransport *transport)
{
    zephyr_transport_params_t *params =
        (zephyr_transport_params_t *)transport->args;

    params->uart_dev = DEVICE_DT_GET(UART_NODE);
    if (!params->uart_dev) return false;
    if (!device_is_ready(params->uart_dev)) return false;

    ring_buf_init(&in_ringbuf, sizeof(uart_in_buffer), uart_in_buffer);
    ring_buf_init(&out_ringbuf, sizeof(uart_out_buffer), uart_out_buffer);

    uart_irq_callback_set(params->uart_dev, uart_fifo_callback);
    uart_irq_rx_enable(params->uart_dev);

    return true;
}

/* ============================================================
 * TRANSPORT CLOSE
 * ============================================================ */
bool zephyr_transport_close(struct uxrCustomTransport *transport)
{
    zephyr_transport_params_t *params =
        (zephyr_transport_params_t *)transport->args;

    if (params->uart_dev != NULL) {
        uart_irq_rx_disable(params->uart_dev);
    }
    return true;
}

/* ============================================================
 * TRANSPORT WRITE
 * ============================================================ */
size_t zephyr_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err)
{
    zephyr_transport_params_t *params =
        (zephyr_transport_params_t *)transport->args;

    for (size_t i = 0; i < len; i++) {
        uart_poll_out(params->uart_dev, buf[i]);
    }

    *err = 0;
    return len;
}

/* ============================================================
 * TRANSPORT READ
 * ============================================================ */
size_t zephyr_transport_read(struct uxrCustomTransport *transport,
                             uint8_t *buf, size_t len,
                             int timeout, uint8_t *err)
{
    zephyr_transport_params_t *params =
        (zephyr_transport_params_t *)transport->args;

    size_t read = 0;
    int spent_time = 0;

    while (ring_buf_is_empty(&in_ringbuf) && spent_time < timeout) {
        k_usleep(1000);
        spent_time++;
    }

    uart_irq_rx_disable(params->uart_dev);
    read = ring_buf_get(&in_ringbuf, buf, len);
    uart_irq_rx_enable(params->uart_dev);

    *err = 0;
    return read;
}