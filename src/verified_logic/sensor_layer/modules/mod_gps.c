#include "../sal_core.h"
#include "../sal_types.h"
#include "common/logger.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* ============================================================
 * SAL MODULE — GPS (u-blox NEO-M8N) — FINAL v3
 * ============================================================
 * Aligned with verified bare-metal working code:
 *   ✅ Full NMEA checksum verify
 *   ✅ Multi-constellation: $GP, $GN, $GL, $GA, $BD
 *   ✅ GGA + RMC + GSV parsing
 *   ✅ Line buffer with overflow handling
 *   ✅ Health = DEGRADED until fix, OK when fix
 * ============================================================ */

#if SAL_ENABLE_GPS

static const struct device *g_gps_uart = NULL;
static bool                 g_gps_init_done = false;

static char                 g_line_buf[SAL_GPS_LINE_BUF_SIZE];
static uint16_t             g_line_pos = 0U;
static uint32_t             g_overflow_count = 0U;

static sal_gps_data_t       g_gps_data;

/* Extra fields from bare-metal code */
static uint8_t  g_sats_in_view = 0U;
static uint8_t  g_rmc_valid    = 0U;
static float    g_hdop         = 0.0f;
static uint32_t g_gga_count    = 0U;
static uint32_t g_rmc_count    = 0U;
static uint32_t g_gsv_count    = 0U;

/* ============================================================
 * NMEA HELPERS
 * ============================================================ */
static float nmea_to_deg(const char *s, const char *dir)
{
    if (!s || !*s) return 0.0f;
    float v = strtof(s, NULL);
    int deg = (int)(v / 100.0f);
    float min = v - (deg * 100.0f);
    float dec = deg + (min / 60.0f);
    if (dir && (*dir == 'S' || *dir == 'W')) dec = -dec;
    return dec;
}

static int nmea_checksum_ok(const char *line)
{
    if (line[0] != '$') return 0;
    const char *star = strchr(line, '*');
    if (!star || star[1] == 0 || star[2] == 0) return 0;

    uint8_t cs = 0;
    for (const char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;

    char hex[3] = { star[1], star[2], 0 };
    uint8_t given = (uint8_t)strtol(hex, NULL, 16);
    return (cs == given);
}

static int split_fields(char *tmp, char **fields, int max_n)
{
    int n = 0;
    char *p = tmp;
    while (p && n < max_n) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = 0;
        p = comma + 1;
    }
    return n;
}

/* ============================================================
 * Multi-constellation prefix check
 * $GPGGA, $GNGGA, $GLGGA, $GAGGA, $BDGGA — all valid
 * ============================================================ */
static bool is_msg_type(const char *line, const char *type)
{
    if (line[0] != '$') return false;
    /* line+1 = talker ID (2 chars), line+3 = msg type (3 chars) */
    return (strncmp(line + 3, type, 3) == 0);
}

/* ============================================================
 * GGA PARSER
 * ============================================================ */
static void parse_gga(const char *line)
{
    g_gga_count++;

    char tmp[SAL_GPS_LINE_BUF_SIZE];
    strncpy(tmp, line, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;

    char *fields[15] = {0};
    int n = split_fields(tmp, fields, 15);
    if (n < 10) return;

    uint8_t fix       = (uint8_t)atoi(fields[6]);
    uint8_t sats_used = (uint8_t)atoi(fields[7]);
    float   hdop      = strtof(fields[8], NULL);
    float   alt       = strtof(fields[9], NULL);

    g_hdop = hdop;

    /* Update GPS data */
    g_gps_data.fix_type   = fix;
    g_gps_data.satellites = sats_used;
    g_gps_data.hdop       = hdop;
    g_gps_data.alt_m      = alt;

    if (fix > 0) {
        g_gps_data.lat_deg = nmea_to_deg(fields[2], fields[3]);
        g_gps_data.lon_deg = nmea_to_deg(fields[4], fields[5]);
    }
}

/* ============================================================
 * RMC PARSER (most reliable for lat/lon)
 * ============================================================ */
static void parse_rmc(const char *line)
{
    g_rmc_count++;

    char tmp[SAL_GPS_LINE_BUF_SIZE];
    strncpy(tmp, line, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;

    char *fields[15] = {0};
    int n = split_fields(tmp, fields, 15);
    if (n < 10) return;

    g_rmc_valid = (fields[2] && fields[2][0] == 'A') ? 1U : 0U;
    if (!g_rmc_valid) return;

    g_gps_data.lat_deg = nmea_to_deg(fields[3], fields[4]);
    g_gps_data.lon_deg = nmea_to_deg(fields[5], fields[6]);

    float speed_knots = strtof(fields[7], NULL);
    g_gps_data.vel_ned[0] = speed_knots * 0.514444f;  /* knots → m/s */
}

/* ============================================================
 * GSV PARSER (satellites in view)
 * ============================================================ */
static void parse_gsv(const char *line)
{
    g_gsv_count++;

    char tmp[SAL_GPS_LINE_BUF_SIZE];
    strncpy(tmp, line, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;

    char *fields[25] = {0};
    int n = split_fields(tmp, fields, 25);
    if (n < 4) return;

    int sv = atoi(fields[3]);
    if (sv > g_sats_in_view) g_sats_in_view = (uint8_t)sv;
}

/* ============================================================
 * MAIN NMEA DISPATCHER
 * ============================================================ */
static void gps_process_line(const char *line)
{
    if (line == NULL || line[0] != '$') return;

    /* ✅ Verify checksum first */
    if (!nmea_checksum_ok(line)) {
        return;
    }

    /* ✅ Multi-constellation dispatch */
    if (is_msg_type(line, "GGA")) {
        parse_gga(line);
    } else if (is_msg_type(line, "RMC")) {
        parse_rmc(line);
    } else if (is_msg_type(line, "GSV")) {
        parse_gsv(line);
    }

    /* Update health */
    if (g_rmc_valid || g_gps_data.fix_type > 0) {
        if (fabsf(g_gps_data.lat_deg) > 0.0001f ||
            fabsf(g_gps_data.lon_deg) > 0.0001f) {
            g_gps_data.health = SAL_HEALTH_OK;
        } else {
            g_gps_data.health = SAL_HEALTH_DEGRADED;
        }
    } else {
        g_gps_data.health = SAL_HEALTH_DEGRADED;   /* data flowing but no fix */
    }

    g_gps_data.timestamp_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());
}

/* ============================================================
 * INIT
 * ============================================================ */
static sal_status_t mod_gps_init(void)
{
    g_gps_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));

    if (!device_is_ready(g_gps_uart)) {
        LOG_E(LOG_TAG_MAIN, "[SAL-GPS] USART1 not ready");
        return SAL_ERR_HW;
    }

    g_line_pos = 0U;
    g_overflow_count = 0U;
    g_sats_in_view = 0U;
    g_rmc_valid = 0U;
    g_hdop = 0.0f;
    g_gga_count = 0U;
    g_rmc_count = 0U;
    g_gsv_count = 0U;

    memset(&g_gps_data, 0, sizeof(g_gps_data));
    g_gps_data.health = SAL_HEALTH_UNKNOWN;

    g_gps_init_done = true;

    LOG_I(LOG_TAG_MAIN, "[SAL-GPS] init OK (NEO-M8N @ USART1 9600)");
    return SAL_OK;
}

/* ============================================================
 * READ — drain ALL bytes, no fixed budget
 * ============================================================ */
static sal_status_t mod_gps_read(void *out)
{
    if (out == NULL || !g_gps_init_done) {
        return SAL_ERR_NOT_INIT;
    }

    sal_gps_data_t *gps_out = (sal_gps_data_t *)out;

    /* Drain ALL available bytes — no fixed budget */
    uint8_t byte;
    uint32_t drained = 0U;
    const uint32_t MAX_DRAIN = 1024U;   /* safety limit */

    while (drained < MAX_DRAIN) {
        if (uart_poll_in(g_gps_uart, &byte) != 0) {
            break;
        }
        drained++;

        if (byte == '\n' || byte == '\r') {
            if (g_line_pos > 0U) {
                g_line_buf[g_line_pos] = '\0';
                gps_process_line(g_line_buf);
                g_line_pos = 0U;
            }
        } else {
            if (g_line_pos < (SAL_GPS_LINE_BUF_SIZE - 1U)) {
                g_line_buf[g_line_pos++] = (char)byte;
            } else {
                /* Overflow — drop partial line, count it */
                g_line_pos = 0U;
                g_overflow_count++;
            }
        }
    }

    *gps_out = g_gps_data;

    /* Return OK if we have any valid data */
    if (g_gps_data.health == SAL_HEALTH_OK ||
        g_gps_data.health == SAL_HEALTH_DEGRADED) {
        return SAL_OK;
    }

    return SAL_ERR;
}

/* ============================================================
 * HEALTH
 * ============================================================ */
static sal_status_t mod_gps_health(void)
{
    if (g_gps_data.health == SAL_HEALTH_OK)       return SAL_OK;
    if (g_gps_data.health == SAL_HEALTH_DEGRADED) return SAL_ERR;
    if (g_gps_data.health == SAL_HEALTH_FAIL)     return SAL_ERR_HW;
    return SAL_ERR_NOT_INIT;
}

/* ============================================================
 * DEINIT
 * ============================================================ */
static void mod_gps_deinit(void)
{
    g_gps_init_done = false;
    g_gps_uart = NULL;
}

/* ============================================================
 * REGISTER
 * ============================================================ */
static const sal_sensor_t g_gps_sensor = {
    .name       = "gps",
    .bus        = SAL_BUS_UART,
    .rate_hz    = SAL_GPS_RATE_HZ,
    .timeout_ms = SAL_GPS_TIMEOUT_MS,
    .init       = mod_gps_init,
    .read       = mod_gps_read,
    .health_fn  = mod_gps_health,
    .deinit     = mod_gps_deinit,
};

__attribute__((constructor))
static void mod_gps_auto_register(void)
{
    (void)sal_register(&g_gps_sensor);
}

#endif /* SAL_ENABLE_GPS */