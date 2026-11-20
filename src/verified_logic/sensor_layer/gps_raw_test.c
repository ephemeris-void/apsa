#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "common/logger.h"

void gps_raw_test_run(void)
{
    const struct device *gps = DEVICE_DT_GET(DT_NODELABEL(usart1));
    if (!device_is_ready(gps)) {
        LOG_E(LOG_TAG_MAIN, "[GPS-RAW] USART1 NOT READY");
        return;
    }

    LOG_I(LOG_TAG_MAIN, "[GPS-RAW] === GPS NMEA Raw Test (3000ms) ===");

    uint8_t byte;
    int count = 0;
    int line = 0;
    int64_t start = k_uptime_get();

    while ((k_uptime_get() - start) < 3000) {
        if (uart_poll_in(gps, &byte) == 0) {
            /* Print as ASCII (readable NMEA) */
            if (count < 500) {
                LOG_I(LOG_TAG_MAIN, "[GPS-RAW] %c", (byte >= 32 && byte < 127) ? byte : '.');
            }
            count++;
            if (byte == '\n') line++;
        }
        k_msleep(1);
    }

    LOG_I(LOG_TAG_MAIN, "[GPS-RAW] Total bytes: %d, lines: %d", count, line);
    LOG_I(LOG_TAG_MAIN, "[GPS-RAW] === Done ===");
}
