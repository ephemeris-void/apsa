#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "common/logger.h"

/* ============================================================
 * LoRa RAW TEST — capture everything from module
 * ============================================================
 * Runs BEFORE sal_init_all() to see:
 *   - Any bytes module sends on power-up (boot message)
 *   - AT command response
 * ============================================================ */

void lora_raw_test_run(void)
{
    const struct device *lora = DEVICE_DT_GET(DT_NODELABEL(uart4));
    if (!device_is_ready(lora)) {
        LOG_E(LOG_TAG_MAIN, "[LORA-RAW] UART4 NOT READY");
        return;
    }

    LOG_I(LOG_TAG_MAIN, "[LORA-RAW] === LoRa Raw Test Starting ===");
    LOG_I(LOG_TAG_MAIN, "[LORA-RAW] Listening for 2000ms (module boot)");

    /* ---------- Phase 1: Passive listen 2s ---------- */
    uint8_t byte;
    int count = 0;
    int64_t start = k_uptime_get();

    while ((k_uptime_get() - start) < 2000) {
        if (uart_poll_in(lora, &byte) == 0) {
            if (count < 80) {
                LOG_I(LOG_TAG_MAIN, "[LORA-RAW] boot RX: 0x%02X '%c'",
                      byte, (byte >= 32 && byte < 127) ? byte : '.');
            }
            count++;
        }
        k_msleep(1);
    }
    LOG_I(LOG_TAG_MAIN, "[LORA-RAW] Passive listen done. Bytes: %d", count);

    /* ---------- Phase 2: Send AT at 9600 ---------- */
    LOG_I(LOG_TAG_MAIN, "[LORA-RAW] Sending 'AT\\r\\n' at 9600 baud");
    const char *cmd = "AT\r\n";
    for (int i = 0; cmd[i] != '\0'; i++) {
        uart_poll_out(lora, cmd[i]);
    }

    count = 0;
    start = k_uptime_get();
    while ((k_uptime_get() - start) < 1500) {
        if (uart_poll_in(lora, &byte) == 0) {
            if (count < 80) {
                LOG_I(LOG_TAG_MAIN, "[LORA-RAW] AT-resp RX: 0x%02X '%c'",
                      byte, (byte >= 32 && byte < 127) ? byte : '.');
            }
            count++;
        }
        k_msleep(1);
    }
    LOG_I(LOG_TAG_MAIN, "[LORA-RAW] AT 9600 response bytes: %d", count);

    LOG_I(LOG_TAG_MAIN, "[LORA-RAW] === Test done ===");
}