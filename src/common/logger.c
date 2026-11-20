/* ============================================================
 * LOGGER — IMPLEMENTATION
 * ============================================================ */

#include "logger.h"
#include "../hal/hal_uart.h"
#include "../hal/hal_time.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ============================================================
 * Internal State
 * ============================================================ */
typedef struct {
    log_level_t   level;
    uint32_t      rate_counter;
    uint32_t      rate_window_start_ms;
    uint32_t      drop_count;
} log_tag_state_t;

static log_level_t    g_global_level = LOG_LEVEL_INFO;
static log_tag_state_t g_tag_state[LOG_TAG_COUNT];
static struct k_mutex  g_log_mutex;
static bool            g_initialized = false;

/* ============================================================
 * Tag Names
 * ============================================================ */
static const char *tag_name(log_tag_t tag)
{
    switch (tag) {
    case LOG_TAG_MAIN:     return "MAIN";
    case LOG_TAG_AGENT:    return "AGENT";
    case LOG_TAG_EKF:      return "EKF";
    case LOG_TAG_AI:       return "AI";
    case LOG_TAG_IMU:      return "IMU";
    case LOG_TAG_ROS:      return "ROS";
    case LOG_TAG_IPC:      return "IPC";
    case LOG_TAG_CONTROL:  return "CTRL";
    case LOG_TAG_HEALTH:   return "HLTH";
    case LOG_TAG_PIPELINE: return "PIPE";
    case LOG_TAG_METRICS:  return "METR";
    default:               return "?";
    }
}

/* ============================================================
 * Level Names
 * ============================================================ */
static char level_char(log_level_t lvl)
{
    switch (lvl) {
    case LOG_LEVEL_ERROR: return 'E';
    case LOG_LEVEL_WARN:  return 'W';
    case LOG_LEVEL_INFO:  return 'I';
    case LOG_LEVEL_DEBUG: return 'D';
    case LOG_LEVEL_TRACE: return 'T';
    default:              return '?';
    }
}

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t logger_init(log_level_t global_level)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    g_global_level = global_level;

    for (uint32_t i = 0U; i < LOG_TAG_COUNT; i++) {
        g_tag_state[i].level = global_level;
        g_tag_state[i].rate_counter = 0U;
        g_tag_state[i].rate_window_start_ms = 0U;
        g_tag_state[i].drop_count = 0U;
    }

    k_mutex_init(&g_log_mutex);
    g_initialized = true;

    return PROJ_OK;
}

/* ============================================================
 * LEVEL CONTROL
 * ============================================================ */
void logger_set_level(log_level_t level)
{
    g_global_level = level;
}

log_level_t logger_get_level(void)
{
    return g_global_level;
}

void logger_set_tag_level(log_tag_t tag, log_level_t level)
{
    if (tag >= LOG_TAG_COUNT) {
        return;
    }
    g_tag_state[tag].level = level;
}

log_level_t logger_get_tag_level(log_tag_t tag)
{
    if (tag >= LOG_TAG_COUNT) {
        return LOG_LEVEL_NONE;
    }
    return g_tag_state[tag].level;
}

/* ============================================================
 * CORE LOG
 * ============================================================ */
void logger_log(log_level_t  level,
                log_tag_t    tag,
                const char  *file,
                uint16_t     line,
                const char  *fmt,
                ...)
{
    if (!g_initialized || tag >= LOG_TAG_COUNT) {
        return;
    }

    if (level > g_tag_state[tag].level) {
        return;
    }

    char buf[192];
    int n = 0;

    /* Timestamp */
    uint32_t now_ms = hal_time_ms_32();

    n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                  "[%5u]%c %-5s ",
                  (unsigned)(now_ms % 100000U),
                  level_char(level),
                  tag_name(tag));

    if (n < 0 || n >= (int)sizeof(buf)) {
        return;
    }

    /* User message */
    va_list args;
    va_start(args, fmt);
    int m = vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, args);
    va_end(args);

    if (m < 0) {
        return;
    }
    n += m;

    /* Newline */
    if (n < (int)sizeof(buf) - 1) {
        buf[n++] = '\n';
        buf[n]   = '\0';
    } else {
        buf[sizeof(buf) - 2] = '\n';
        buf[sizeof(buf) - 1] = '\0';
        n = (int)sizeof(buf) - 1;
    }

    /* Non-blocking write */
    (void)hal_uart_write_async(HAL_UART_TELEMETRY,
                               (const uint8_t *)buf,
                               (size_t)n);

    (void)file;
    (void)line;
}

/* ============================================================
 * RATE-LIMITED LOG
 * ============================================================ */
void logger_log_ratelimited(log_level_t  level,
                            log_tag_t    tag,
                            uint32_t     max_per_sec,
                            const char  *file,
                            uint16_t     line,
                            const char  *fmt,
                            ...)
{
    if (!g_initialized || tag >= LOG_TAG_COUNT) {
        return;
    }

    if (level > g_tag_state[tag].level) {
        return;
    }

    if (max_per_sec == 0U) {
        return;
    }

    uint32_t now_ms = hal_time_ms_32();
    log_tag_state_t *st = &g_tag_state[tag];

    /* Window reset each second */
    if ((now_ms - st->rate_window_start_ms) >= 1000U) {
        st->rate_window_start_ms = now_ms;
        st->rate_counter = 0U;
    }

    if (st->rate_counter >= max_per_sec) {
        st->drop_count++;
        return;
    }

    st->rate_counter++;

    /* Forward to core log */
    char buf[192];
    int n = 0;

    n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                  "[%5u]%c %-5s ",
                  (unsigned)(now_ms % 100000U),
                  level_char(level),
                  tag_name(tag));

    if (n < 0 || n >= (int)sizeof(buf)) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    int m = vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, args);
    va_end(args);

    if (m < 0) {
        return;
    }
    n += m;

    if (n < (int)sizeof(buf) - 1) {
        buf[n++] = '\n';
        buf[n]   = '\0';
    } else {
        buf[sizeof(buf) - 2] = '\n';
        buf[sizeof(buf) - 1] = '\0';
        n = (int)sizeof(buf) - 1;
    }

    (void)hal_uart_write_async(HAL_UART_TELEMETRY,
                               (const uint8_t *)buf,
                               (size_t)n);

    (void)file;
    (void)line;
}

/* ============================================================
 * BINARY LOG
 * ============================================================ */
void logger_binary(log_tag_t   tag,
                   const void *data,
                   size_t      len)
{
    if (!g_initialized || data == NULL || len == 0U) {
        return;
    }

    /* Binary framing: [0xAA][tag][len_hi][len_lo][payload...][checksum] */
    uint8_t header[4];
    header[0] = 0xAAU;
    header[1] = (uint8_t)tag;
    header[2] = (uint8_t)((len >> 8) & 0xFFU);
    header[3] = (uint8_t)(len & 0xFFU);

    (void)hal_uart_write_async(HAL_UART_TELEMETRY, header, sizeof(header));
    (void)hal_uart_write_async(HAL_UART_TELEMETRY,
                               (const uint8_t *)data,
                               len);
}

/* ============================================================
 * DUMP STATS
 * ============================================================ */
void logger_dump_stats(void)
{
    if (!g_initialized) {
        return;
    }

    char buf[128];
    int n = snprintf(buf, sizeof(buf), "=== LOGGER STATS ===\n");
    (void)hal_uart_write_async(HAL_UART_TELEMETRY,
                               (const uint8_t *)buf, (size_t)n);

    for (uint32_t i = 0U; i < LOG_TAG_COUNT; i++) {
        if (g_tag_state[i].drop_count == 0U) {
            continue;
        }
        n = snprintf(buf, sizeof(buf),
                     "%-5s dropped: %u\n",
                     tag_name((log_tag_t)i),
                     (unsigned)g_tag_state[i].drop_count);
        (void)hal_uart_write_async(HAL_UART_TELEMETRY,
                                   (const uint8_t *)buf, (size_t)n);
    }
}