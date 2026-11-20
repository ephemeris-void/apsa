#include "sal_diag.h"
#include "common/logger.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>

void sal_diag_run(void)
{
    LOG_I(LOG_TAG_MAIN, "========================================");
    LOG_I(LOG_TAG_MAIN, "[DIAG] ROOT CAUSE DETECTOR STARTING");
    LOG_I(LOG_TAG_MAIN, "========================================");

    /* ---------- I2C1 (BARO) ---------- */
    LOG_I(LOG_TAG_MAIN, "[DIAG-1] Scanning I2C1 (BARO)...");
    const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c1)) {
        LOG_E(LOG_TAG_MAIN, "[DIAG-1] I2C1 NOT READY");
    } else {
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            struct i2c_msg msg;
            uint8_t dummy = 0;
            msg.buf = &dummy;
            msg.len = 1;
            msg.flags = I2C_MSG_READ | I2C_MSG_STOP;
            if (i2c_transfer(i2c1, &msg, 1, addr) == 0) {
                LOG_I(LOG_TAG_MAIN, "[DIAG-1] FOUND @ 0x%02X", addr);
            }
        }
    }

    /* ---------- I2C4 (MAG) extended ---------- */
    LOG_I(LOG_TAG_MAIN, "[DIAG-2] Extended scan I2C4 (MAG)...");
    const struct device *i2c4 = DEVICE_DT_GET(DT_NODELABEL(i2c4));
    if (!device_is_ready(i2c4)) {
        LOG_E(LOG_TAG_MAIN, "[DIAG-2] I2C4 NOT READY");
    } else {
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            struct i2c_msg msg;
            uint8_t dummy = 0;
            msg.buf = &dummy;
            msg.len = 1;
            msg.flags = I2C_MSG_READ | I2C_MSG_STOP;
            if (i2c_transfer(i2c4, &msg, 1, addr) == 0) {
                LOG_I(LOG_TAG_MAIN, "[DIAG-2] *** FOUND @ 0x%02X ***", addr);

                /* Try common ID registers */
                for (uint8_t reg = 0x00; reg <= 0x0D; reg++) {
                    uint8_t val = 0;
                    if (i2c_write_read(i2c4, addr, &reg, 1, &val, 1) == 0) {
                        LOG_I(LOG_TAG_MAIN, "[DIAG-2]   Reg[0x%02X] = 0x%02X", reg, val);
                    }
                }
            }
        }
    }

    /* ---------- GPS UART1 ---------- */
    LOG_I(LOG_TAG_MAIN, "[DIAG-3] GPS UART1 byte test (1000ms)...");
    const struct device *gps = DEVICE_DT_GET(DT_NODELABEL(usart1));
    if (!device_is_ready(gps)) {
        LOG_E(LOG_TAG_MAIN, "[DIAG-3] USART1 NOT READY");
    } else {
        uint8_t byte;
        int count = 0;
        int64_t start = k_uptime_get();
        while ((k_uptime_get() - start) < 1000) {
            if (uart_poll_in(gps, &byte) == 0) {
                if (count < 20) {
                    LOG_I(LOG_TAG_MAIN, "[DIAG-3] RX: 0x%02X '%c'",
                          byte, (byte >= 32 && byte < 127) ? byte : '.');
                }
                count++;
            }
            k_msleep(1);
        }
        LOG_I(LOG_TAG_MAIN, "[DIAG-3] Total bytes: %d", count);
    }

    /* ---------- LoRa UART4 ---------- */
    LOG_I(LOG_TAG_MAIN, "[DIAG-4] LoRa UART4 AT test...");
    const struct device *lora = DEVICE_DT_GET(DT_NODELABEL(uart4));
    if (!device_is_ready(lora)) {
        LOG_E(LOG_TAG_MAIN, "[DIAG-4] UART4 NOT READY");
    } else {
        uint8_t dummy;
        while (uart_poll_in(lora, &dummy) == 0) { }

        const char *cmd = "AT\r\n";
        for (int i = 0; cmd[i] != '\0'; i++) {
            uart_poll_out(lora, cmd[i]);
        }

        uint8_t byte;
        int count = 0;
        int64_t start = k_uptime_get();
        while ((k_uptime_get() - start) < 1500) {
            if (uart_poll_in(lora, &byte) == 0) {
                if (count < 30) {
                    LOG_I(LOG_TAG_MAIN, "[DIAG-4] RX: 0x%02X '%c'",
                          byte, (byte >= 32 && byte < 127) ? byte : '.');
                }
                count++;
            }
            k_msleep(1);
        }
        LOG_I(LOG_TAG_MAIN, "[DIAG-4] Response bytes: %d", count);
    }

    LOG_I(LOG_TAG_MAIN, "========================================");
    LOG_I(LOG_TAG_MAIN, "[DIAG] DONE");
    LOG_I(LOG_TAG_MAIN, "========================================");
}