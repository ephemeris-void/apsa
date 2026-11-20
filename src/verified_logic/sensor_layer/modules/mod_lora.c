#include "../sal_core.h"
#include "../sal_types.h"
#include "common/logger.h"
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

#if SAL_ENABLE_LORA
#ifndef SAL_LORA_MY_ADDR
#define SAL_LORA_MY_ADDR    1
#endif
#ifndef SAL_LORA_PEER_ADDR
#define SAL_LORA_PEER_ADDR  2
#endif
#define LORA_FREQ_HZ        "915000000"
#define LORA_EIRP_DBM       "22"
#define LORA_BW_HZ          "125000"
#define LORA_SF             "7"
#define RECV_PFX            "+RECV="
#define RECV_PFX_LEN        6U
#define LORA_MAX_PAYLOAD    255U
#define LORA_TX_MAX         100U
#define LORA_RESYNC_MS      400U
#define LORA_THREAD_STACK   2048
#define LORA_THREAD_PRIO    8
#define LORA_POLL_MS        5

/* ============================================================
 * Global flags
 * ============================================================ */
volatile bool g_lora_just_received = false;

/* ============================================================
 * LED debug
 * ============================================================ */
static const struct gpio_dt_spec led_green  = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec led_yellow = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});
static const struct gpio_dt_spec led_red    = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0});

static bool led_usable(const struct gpio_dt_spec *led) {
    if (led->port == NULL) return false;
    if (led->port == DEVICE_DT_GET(DT_NODELABEL(gpioa)) && led->pin == 5U) return false;
    return true;
}
static void led_init_one(const struct gpio_dt_spec *led, const char *name) {
    if (led_usable(led)) gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
    else if (led->port != NULL) printk("[SAL-LORA] LED '%s' is on PA5 - skipped\n", name);
}
static void led_init(void) { led_init_one(&led_green, "green"); led_init_one(&led_yellow, "yellow"); led_init_one(&led_red, "red"); }
static void led_set(const struct gpio_dt_spec *led, int on) { if (led_usable(led)) gpio_pin_set_dt(led, on ? 1 : 0); }

/* ------------------------------------------------------------ state */
static const struct device *g_lora_uart = NULL;
static bool                 g_lora_init_done = false;
static sal_lora_data_t      g_lora_data;

RING_BUF_DECLARE(g_lora_rb, 1024);
static volatile uint32_t    g_rb_overflow = 0U;
static volatile uint32_t    g_isr_byte_count = 0U;

K_MUTEX_DEFINE(g_lora_mtx);

enum rx_state { ST_HUNT, ST_HDR, ST_PAYLOAD };
static enum rx_state g_st = ST_HUNT;
static uint8_t       g_match = 0U;
static uint8_t       g_hdr[5];
static uint8_t       g_hdr_n = 0U;
static uint8_t       g_pay[LORA_MAX_PAYLOAD];
static uint16_t      g_pay_n = 0U;
static uint16_t      g_pay_len = 0U;
static char          g_line[96];
static uint8_t       g_line_n = 0U;
static uint32_t      g_last_byte_ms = 0U;
static uint32_t      g_bytes_total = 0U;

static const char   *g_expect = NULL;
static volatile bool g_hit = false;
static volatile bool g_err = false;

/* ============================================================
 * ✅ RAW byte tracker — ISR-এর সব বাইট ধরে রাখে
 * ============================================================ */
#define RAW_BUF_SIZE 128
static uint8_t  g_raw_buf[RAW_BUF_SIZE];
static volatile uint32_t g_raw_n = 0U;
static volatile uint32_t g_raw_total = 0U;

static void raw_reset(void) { g_raw_n = 0U; }

static void raw_dump(const char *tag)
{
    LOG_W(LOG_TAG_MAIN, "[SAL-LORA] %s RAW (%u bytes):", tag, (unsigned)g_raw_n);
    for (uint32_t i = 0U; i < g_raw_n && i < RAW_BUF_SIZE; i++) {
        LOG_W(LOG_TAG_MAIN, "  [%u] 0x%02X '%c'", (unsigned)i,
              (unsigned char)g_raw_buf[i],
              (g_raw_buf[i] >= 32 && g_raw_buf[i] < 127) ? g_raw_buf[i] : '.');
    }
}

/* ------------------------------------------------------------ ISR */
static void lora_uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t tmp[32];

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            int n = uart_fifo_read(dev, tmp, sizeof(tmp));
            if (n > 0) {
                g_isr_byte_count += (uint32_t)n;
                g_raw_total += (uint32_t)n;

                /* ✅ ISR-এ প্রতিটা বাইট raw buffer-এ */
                for (int i = 0; i < n; i++) {
                    if (g_raw_n < RAW_BUF_SIZE) {
                        g_raw_buf[g_raw_n++] = tmp[i];
                    }
                }

                uint32_t put = ring_buf_put(&g_lora_rb, tmp, (uint32_t)n);
                if (put < (uint32_t)n) {
                    g_rb_overflow += ((uint32_t)n - put);
                }
            }
        }
    }
}

/* ------------------------------------------------------------ parser */
static void parser_reset(void)
{
    g_st = ST_HUNT;
    g_match = 0U;
    g_hdr_n = 0U;
    g_pay_n = 0U;
    g_pay_len = 0U;
    g_line_n = 0U;
    g_line[0] = '\0';
}

static void line_check(void)
{
    if (g_expect != NULL && strstr(g_line, g_expect) != NULL) {
        LOG_I(LOG_TAG_MAIN, "[SAL-LORA] >>> LINE HIT: '%s' (expect='%s')",
              g_line, g_expect);
        g_hit = true;
    }
    if (strstr(g_line, "ERROR") != NULL || strstr(g_line, "NOCOMMAND") != NULL) {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] >>> ERROR LINE: '%s'", g_line);
        g_err = true;
    }
}

static void frame_deliver(void)
{
    uint8_t to   = g_hdr[0];
    uint8_t from = g_hdr[1];
    int rssi     = -(int)g_hdr[2];
    int snr      = (int)g_hdr[3] - 50;

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] FRAME: to=%u from=%u rssi=%d snr=%d len=%u",
          (unsigned)to, (unsigned)from, rssi, snr, (unsigned)g_pay_len);

    if (to != 0xFFU && to != (uint8_t)SAL_LORA_MY_ADDR) {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] RX ignored (to=%u)", (unsigned)to);
        return;
    }

    uint16_t copy_len = g_pay_len;
    if (copy_len > SAL_LORA_RX_BUF_SIZE) {
        copy_len = SAL_LORA_RX_BUF_SIZE;
    }
    memcpy(g_lora_data.rx_buffer, g_pay, copy_len);
    g_lora_data.rx_len = copy_len;
    g_lora_data.rx_count++;
    g_lora_data.rx_timestamp_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());

    g_lora_just_received = true;

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] RX from=%u to=%u rssi=%d snr=%d len=%u",
          (unsigned)from, (unsigned)to, rssi, snr, (unsigned)g_pay_len);
}

static void parser_feed(uint8_t b)
{
    g_bytes_total++;

    switch (g_st) {
    case ST_HUNT:
        if (b == (uint8_t)RECV_PFX[g_match]) {
            g_match++;
            if (g_match == RECV_PFX_LEN) {
                LOG_I(LOG_TAG_MAIN, "[SAL-LORA] +RECV= detected, entering ST_HDR");
                g_match = 0U;
                g_hdr_n = 0U;
                g_line_n = 0U;
                g_line[0] = '\0';
                g_st = ST_HDR;
                return;
            }
        } else {
            g_match = (b == (uint8_t)'+') ? 1U : 0U;
        }

        if (b == (uint8_t)'\n') {
            line_check();
            g_line_n = 0U;
            g_line[0] = '\0';
        } else if (b != (uint8_t)'\r') {
            if (g_line_n < (sizeof(g_line) - 1U)) {
                g_line[g_line_n++] = (char)b;
                g_line[g_line_n] = '\0';
                line_check();
            }
        }
        break;

    case ST_HDR:
        g_hdr[g_hdr_n++] = b;
        if (g_hdr_n >= sizeof(g_hdr)) {
            g_pay_len = g_hdr[4];
            g_pay_n = 0U;
            LOG_I(LOG_TAG_MAIN, "[SAL-LORA] HDR complete: pay_len=%u", (unsigned)g_pay_len);
            if (g_pay_len == 0U || g_pay_len > LORA_MAX_PAYLOAD) {
                LOG_W(LOG_TAG_MAIN, "[SAL-LORA] HDR invalid, back to ST_HUNT");
                g_st = ST_HUNT;
            } else {
                g_st = ST_PAYLOAD;
            }
        }
        break;

    case ST_PAYLOAD:
        g_pay[g_pay_n++] = b;
        if (g_pay_n >= g_pay_len) {
            frame_deliver();
            g_st = ST_HUNT;
        }
        break;
    }
}

static void lora_pump(void)
{
    uint8_t b;
    uint32_t budget = 512U;
    uint32_t pumped = 0U;

    while (budget-- > 0U && ring_buf_get(&g_lora_rb, &b, 1U) == 1U) {
        g_last_byte_ms = k_uptime_get_32();
        parser_feed(b);
        pumped++;
    }

    if (pumped > 0U) {
        LOG_D(LOG_TAG_MAIN, "[SAL-LORA] pump: %u bytes, state=%d, line='%s'",
              (unsigned)pumped, (int)g_st, g_line);
    }

    if (g_st != ST_HUNT &&
        (k_uptime_get_32() - g_last_byte_ms) > LORA_RESYNC_MS) {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] frame timeout, resync");
        g_st = ST_HUNT;
        g_match = 0U;
    }
}

/* ------------------------------------------------------------ TX helpers */
static void lora_write(const char *s, size_t n)
{
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] TX >>> %u bytes", (unsigned)n);
    for (size_t i = 0U; i < n; i++) {
        LOG_I(LOG_TAG_MAIN, "  TX[%u] 0x%02X '%c'", (unsigned)i,
              (unsigned char)s[i],
              (s[i] >= 32 && s[i] < 127) ? s[i] : '.');
    }
    for (size_t i = 0U; i < n; i++) {
        uart_poll_out(g_lora_uart, (unsigned char)s[i]);
    }
}

static bool wait_ack(uint32_t timeout_ms)
{
    uint32_t t0 = k_uptime_get_32();
    uint32_t pump_count = 0U;

    while ((k_uptime_get_32() - t0) < timeout_ms) {
        lora_pump();
        pump_count++;
        if (g_hit) {
            LOG_I(LOG_TAG_MAIN, "[SAL-LORA] wait_ack: HIT after %u ms, %u pumps",
                  (unsigned)(k_uptime_get_32() - t0), (unsigned)pump_count);
            return true;
        }
        if (g_err) {
            LOG_W(LOG_TAG_MAIN, "[SAL-LORA] wait_ack: ERR after %u ms",
                  (unsigned)(k_uptime_get_32() - t0));
            return false;
        }
        k_msleep(2);
    }
    LOG_W(LOG_TAG_MAIN, "[SAL-LORA] wait_ack: TIMEOUT %u ms, %u pumps, isr=%u, rb_ovf=%u",
          (unsigned)timeout_ms, (unsigned)pump_count,
          (unsigned)g_isr_byte_count, (unsigned)g_rb_overflow);
    return false;
}

static bool lora_cmd(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    char out[320];
    int n = snprintf(out, sizeof(out), "%s\r\n", cmd);
    if (n <= 0 || (size_t)n >= sizeof(out)) {
        LOG_E(LOG_TAG_MAIN, "[SAL-LORA] lora_cmd: snprintf fail");
        return false;
    }

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] ===== CMD '%s' (expect='%s', timeout=%u ms) =====",
          cmd, expect, (unsigned)timeout_ms);

    g_expect = NULL;
    lora_pump();
    if (g_st == ST_HUNT) {
        g_line_n = 0U;
        g_line[0] = '\0';
    }

    uint32_t bytes_before = g_bytes_total;
    uint32_t raw_before   = g_raw_total;
    raw_reset();
    g_hit = false;
    g_err = false;
    g_expect = expect;

    lora_write(out, (size_t)n);

    bool ok = wait_ack(timeout_ms);
    g_expect = NULL;

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] CMD '%s' => %s (new_bytes=%u, raw_bytes=%u)",
          cmd, ok ? "OK" : "FAIL",
          (unsigned)(g_bytes_total - bytes_before),
          (unsigned)(g_raw_total - raw_before));

    if (!ok) {
        LOG_W(LOG_TAG_MAIN,
              "[SAL-LORA] FAIL cmd='%s' expect='%s' err=%d rx_bytes=%u last_line='%s'",
              cmd, expect, (int)g_err,
              (unsigned)(g_bytes_total - bytes_before), g_line);

        raw_dump("FAIL");

        if (g_line_n > 0U) {
            LOG_W(LOG_TAG_MAIN, "[SAL-LORA] last_line hex (%u bytes):", (unsigned)g_line_n);
            for (uint8_t i = 0U; g_line[i] != '\0' && i < 32U; i++) {
                LOG_W(LOG_TAG_MAIN, "  [%u] 0x%02X '%c'", i,
                      (unsigned char)g_line[i],
                      (g_line[i] >= 32 && g_line[i] < 127) ? g_line[i] : '.');
            }
        }
    }
    return ok;
}

static void lora_send_only(const char *cmd)
{
    char out[64];
    int n = snprintf(out, sizeof(out), "%s\r\n", cmd);
    if (n > 0) {
        LOG_I(LOG_TAG_MAIN, "[SAL-LORA] send_only: '%s' (%d bytes)", cmd, n);
        lora_write(out, (size_t)n);
    }
}

/* ============================================================ poll thread */
static K_THREAD_STACK_DEFINE(lora_thread_stack, LORA_THREAD_STACK);
static struct k_thread lora_thread_data;

static void lora_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    k_thread_name_set(k_current_get(), "lora_poll");
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] poll thread started");

    while (1) {
        if (g_lora_init_done) {
            if (k_mutex_lock(&g_lora_mtx, K_MSEC(10)) == 0) {
                lora_pump();
                k_mutex_unlock(&g_lora_mtx);
            }
        }
        k_msleep(LORA_POLL_MS);
    }
}

/* ------------------------------------------------------------ INIT */
static sal_status_t mod_lora_init(void)
{
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] ==========================================");
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] mod_lora_init START");
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] ==========================================");

    led_init();
    led_set(&led_green, 1);
    k_msleep(200);

    g_lora_uart = DEVICE_DT_GET(DT_NODELABEL(uart4));
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] device ptr = %p", (void *)g_lora_uart);

    if (!device_is_ready(g_lora_uart)) {
        LOG_E(LOG_TAG_MAIN, "[SAL-LORA] UART4 not ready");
        led_set(&led_green, 0);
        led_set(&led_red, 1);
        return SAL_ERR_HW;
    }

    led_set(&led_yellow, 1);
    LOG_I(LOG_TAG_MAIN, "[LED] UART4 ready");

    memset(&g_lora_data, 0, sizeof(g_lora_data));
    g_lora_data.health = SAL_HEALTH_UNKNOWN;
    ring_buf_reset(&g_lora_rb);
    parser_reset();
    raw_reset();
    g_last_byte_ms = k_uptime_get_32();

    uart_irq_callback_user_data_set(g_lora_uart, lora_uart_isr, NULL);
    uart_irq_rx_enable(g_lora_uart);
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] IRQ callback set + RX IRQ enabled");

    k_mutex_lock(&g_lora_mtx, K_FOREVER);

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 1: pre-reboot wait 2000 ms");
    k_msleep(2000);

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 2: send AT+REBOOT (blind)");
    lora_send_only("AT+REBOOT");

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 3: post-reboot wait 3000 ms");
    k_msleep(3000);

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 4: pump + reset parser after reboot");
    lora_pump();
    parser_reset();
    raw_reset();

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 5: AT handshake (10 retries)");
    bool at_ok = false;
    for (uint32_t retry = 0U; retry < 10U; retry++) {
        LOG_I(LOG_TAG_MAIN, "[SAL-LORA] AT retry %u / 10", (unsigned)retry);
        if (lora_cmd("AT", "OK", 1000U)) {
            at_ok = true;
            LOG_I(LOG_TAG_MAIN, "[SAL-LORA] AT OK (retry %u)", (unsigned)retry);
            break;
        }
        k_msleep(300);
    }
    if (!at_ok) {
        LOG_E(LOG_TAG_MAIN, "[SAL-LORA] AT no response (isr_bytes=%u rb_overflow=%u raw_total=%u)",
              (unsigned)g_isr_byte_count, (unsigned)g_rb_overflow, (unsigned)g_raw_total);
        led_set(&led_green, 0);
        led_set(&led_yellow, 0);
        led_set(&led_red, 1);
        g_lora_data.health = SAL_HEALTH_FAIL;
        k_mutex_unlock(&g_lora_mtx);
        LOG_E(LOG_TAG_MAIN, "[SAL-LORA] mod_lora_init END (FAIL at AT)");
        return SAL_ERR_HW;
    }

    led_set(&led_red, 0);
    LOG_I(LOG_TAG_MAIN, "[LED] AT OK");

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 6: AT+LORAMODE=LORA");
    if (!lora_cmd("AT+LORAMODE=LORA", "OK", 2000U)) {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] LORAMODE ack not seen (continuing)");
    }
    k_msleep(200);

    static const struct { const char *cmd; const char *expect; uint32_t to; } seq[] = {
        { "AT+LORAADDR=" STRINGIFY(SAL_LORA_MY_ADDR), "+LORAADDR=OK", 2000U },
        { "AT+FREQS=" LORA_FREQ_HZ,                   "+FREQS=OK",    2000U },
        { "AT+EIRP=" LORA_EIRP_DBM,                   "+EIRP=OK",     2000U },
        { "AT+BW=" LORA_BW_HZ,                        "+BW=OK",       2000U },
        { "AT+SF=" LORA_SF,                           "+SF=OK",       2000U },
        { "AT+JOIN=1",                                "+JOIN=OK",     3000U },
    };

    for (size_t i = 0U; i < ARRAY_SIZE(seq); i++) {
        LOG_I(LOG_TAG_MAIN, "[SAL-LORA] STEP 7.%u: '%s'", (unsigned)(i + 1U), seq[i].cmd);
        bool ok = lora_cmd(seq[i].cmd, seq[i].expect, seq[i].to);
        if (!ok) {
            LOG_W(LOG_TAG_MAIN, "[SAL-LORA] retry '%s'", seq[i].cmd);
            k_msleep(300);
            ok = lora_cmd(seq[i].cmd, seq[i].expect, seq[i].to);
        }
        if (!ok) {
            LOG_E(LOG_TAG_MAIN, "[SAL-LORA] init failed at '%s'", seq[i].cmd);
            led_set(&led_green, 0);
            led_set(&led_yellow, 0);
            led_set(&led_red, 1);
            g_lora_data.health = SAL_HEALTH_FAIL;
            k_mutex_unlock(&g_lora_mtx);
            LOG_E(LOG_TAG_MAIN, "[SAL-LORA] mod_lora_init END (FAIL at %s)", seq[i].cmd);
            return SAL_ERR_HW;
        }
        LOG_I(LOG_TAG_MAIN, "[SAL-LORA] %s OK", seq[i].cmd);
        k_msleep(200);
    }

    g_lora_init_done = true;
    g_lora_data.health = SAL_HEALTH_OK;
    k_mutex_unlock(&g_lora_mtx);

    k_thread_create(&lora_thread_data, lora_thread_stack,
                    K_THREAD_STACK_SIZEOF(lora_thread_stack),
                    lora_thread_entry, NULL, NULL, NULL,
                    LORA_THREAD_PRIO, 0, K_NO_WAIT);

    led_set(&led_red, 0);

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] mod_lora_init END (OK) addr=%u peer=%u isr_bytes=%u",
          (unsigned)SAL_LORA_MY_ADDR, (unsigned)SAL_LORA_PEER_ADDR,
          (unsigned)g_isr_byte_count);
    return SAL_OK;
}

/* ------------------------------------------------------------ READ */
static sal_status_t mod_lora_read(void *out)
{
    if (out == NULL || !g_lora_init_done) {
        return SAL_ERR_NOT_INIT;
    }

    *(sal_lora_data_t *)out = g_lora_data;
    return SAL_OK;
}

/* ------------------------------------------------------------ SEND */
sal_status_t sal_lora_send_raw(const uint8_t *data, uint16_t len)
{
    static const char HEXCH[] = "0123456789ABCDEF";

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] send_raw: len=%u, init_done=%d",
          (unsigned)len, (int)g_lora_init_done);

    if (!g_lora_init_done || data == NULL || len == 0U) {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] send_raw: rejected");
        return SAL_ERR_INVALID;
    }
    if (len > LORA_TX_MAX) len = LORA_TX_MAX;
    if (len > SAL_LORA_TX_BUF_SIZE) len = SAL_LORA_TX_BUF_SIZE;

    char cmd[256];
    int pos = snprintf(cmd, sizeof(cmd), "AT+SEND=%02X%02X",
                       (unsigned)SAL_LORA_PEER_ADDR, (unsigned)SAL_LORA_MY_ADDR);
    if (pos < 0) return SAL_ERR_INVALID;

    for (uint16_t i = 0U; i < len; i++) {
        cmd[pos++] = HEXCH[data[i] >> 4];
        cmd[pos++] = HEXCH[data[i] & 0x0FU];
    }
    cmd[pos] = '\0';

    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] send_raw cmd='%s'", cmd);

    if (k_mutex_lock(&g_lora_mtx, K_MSEC(5000)) != 0) {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] send_raw: mutex timeout");
        return SAL_ERR_HW;
    }
    bool ok = lora_cmd(cmd, "SEND=QUEUE", 3000U);
    if (ok) {
        g_lora_data.tx_count++;
        LOG_I(LOG_TAG_MAIN, "[SAL-LORA] TX: %s", cmd);
    } else {
        LOG_W(LOG_TAG_MAIN, "[SAL-LORA] TX FAIL: %s", cmd);
    }
    k_mutex_unlock(&g_lora_mtx);

    return ok ? SAL_OK : SAL_ERR_HW;
}

/* ------------------------------------------------------------ HEALTH / DEINIT */
static sal_status_t mod_lora_health(void)
{
    return (g_lora_data.health == SAL_HEALTH_OK) ? SAL_OK : SAL_ERR_HW;
}

static void mod_lora_deinit(void)
{
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] deinit");
    if (g_lora_init_done && g_lora_uart != NULL) {
        lora_send_only("AT+REBOOT");
        uart_irq_rx_disable(g_lora_uart);
    }
    led_set(&led_green, 0);
    led_set(&led_yellow, 0);
    led_set(&led_red, 0);
    g_lora_init_done = false;
    g_lora_uart = NULL;
}

/* ------------------------------------------------------------ REGISTER */
static const sal_sensor_t g_lora_sensor = {
    .name       = "lora",
    .bus        = SAL_BUS_UART,
    .rate_hz    = SAL_LORA_RATE_HZ,
    .timeout_ms = SAL_LORA_TIMEOUT_MS,
    .init       = mod_lora_init,
    .read       = mod_lora_read,
    .health_fn  = mod_lora_health,
    .deinit     = mod_lora_deinit,
};

__attribute__((constructor))
static void mod_lora_auto_register(void)
{
    LOG_I(LOG_TAG_MAIN, "[SAL-LORA] auto_register() called");
    (void)sal_register(&g_lora_sensor);
}

#endif /* SAL_ENABLE_LORA */