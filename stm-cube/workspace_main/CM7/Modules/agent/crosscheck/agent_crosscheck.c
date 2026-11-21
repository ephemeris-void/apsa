#include "agent_crosscheck.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT CROSSCHECK — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 30 µs per run
 * ============================================================ */

/* PERF: stats in DTCM (zero wait state) */
static AGENT_DTCM agent_crosscheck_stats_t g_stats;

/* ---------- Median of 3 ---------- */
AGENT_INLINE_FAST int16_t median3(int16_t a, int16_t b, int16_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

/* ---------- Median of N (N ≤ 4) ---------- */
static int16_t median_of(const int16_t *v, uint32_t n)
{
    if (n == 1U) return v[0];
    if (n == 2U) return (int16_t)(((int32_t)v[0] + (int32_t)v[1]) / 2);
    if (n == 3U) return median3(v[0], v[1], v[2]);

    /* n == 4: sort 4 small values */
    int16_t tmp[AGENT_CROSSCHECK_MAX_CHANNELS];
    memcpy(tmp, v, n * sizeof(int16_t));

    for (uint32_t i = 0U; i < n; i++) {
        for (uint32_t j = i + 1U; j < n; j++) {
            if (tmp[j] < tmp[i]) {
                int16_t t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    return (int16_t)(((int32_t)tmp[n/2U - 1U] + (int32_t)tmp[n/2U]) / 2);
}

/* ---------- Vote per channel ---------- */
static uint32_t vote_channel(const int16_t *v, uint32_t n,
                             int16_t median, int16_t max_delta,
                             int16_t *out_value, int16_t *out_max_delta)
{
    uint32_t valid_mask  = 0U;
    uint32_t valid_count = 0U;
    int32_t  sum         = 0;
    int16_t  worst       = 0;

    for (uint32_t i = 0U; i < n; i++) {
        int32_t d = (int32_t)v[i] - (int32_t)median;
        if (d < 0) d = -d;

        if ((int16_t)d > worst) worst = (int16_t)d;

        if (d <= max_delta) {
            valid_mask |= (1UL << i);
            sum += v[i];
            valid_count++;
        }
    }

    if (valid_count > 0U) {
        *out_value = (int16_t)(sum / (int32_t)valid_count);
    } else {
        *out_value = median;
    }
    *out_max_delta = worst;

    return valid_mask;
}

/* ---------- INIT ---------- */
agent_status_t agent_crosscheck_init(void)
{
    memset(&g_stats, 0, sizeof(g_stats));    /* zero-init incl. reserved[] */
    g_stats.last_check_us = agent_time_now_us();
    return AGENT_OK;
}

agent_status_t agent_crosscheck_reset(void)
{
    return agent_crosscheck_init();
}

/* ---------- RUN ---------- */
agent_crosscheck_result_t agent_crosscheck_run(const agent_crosscheck_input_t *in,
                                               agent_crosscheck_output_t *out)
{
    if (in == NULL || out == NULL) {
        return AGENT_CROSSCHECK_FAIL;
    }

    uint32_t n = in->count;

    if (n < AGENT_CROSSCHECK_VOTE_MIN || n > AGENT_CROSSCHECK_MAX_CHANNELS) {
        return AGENT_CROSSCHECK_FAIL;
    }

    int16_t m_ax = median_of(in->ax, n);
    int16_t m_ay = median_of(in->ay, n);
    int16_t m_az = median_of(in->az, n);
    int16_t m_gx = median_of(in->gx, n);
    int16_t m_gy = median_of(in->gy, n);
    int16_t m_gz = median_of(in->gz, n);

    int16_t d_ax, d_ay, d_az, d_gx, d_gy, d_gz;

    uint32_t v_ax = vote_channel(in->ax, n, m_ax, AGENT_CROSSCHECK_DIVERGE_LSB, &out->ax, &d_ax);
    uint32_t v_ay = vote_channel(in->ay, n, m_ay, AGENT_CROSSCHECK_DIVERGE_LSB, &out->ay, &d_ay);
    uint32_t v_az = vote_channel(in->az, n, m_az, AGENT_CROSSCHECK_DIVERGE_LSB, &out->az, &d_az);
    uint32_t v_gx = vote_channel(in->gx, n, m_gx, AGENT_CROSSCHECK_DIVERGE_LSB, &out->gx, &d_gx);
    uint32_t v_gy = vote_channel(in->gy, n, m_gy, AGENT_CROSSCHECK_DIVERGE_LSB, &out->gy, &d_gy);
    uint32_t v_gz = vote_channel(in->gz, n, m_gz, AGENT_CROSSCHECK_DIVERGE_LSB, &out->gz, &d_gz);

    uint32_t valid_mask   = v_ax & v_ay & v_az & v_gx & v_gy & v_gz;
    uint32_t outlier_mask = (~valid_mask) & ((1UL << n) - 1UL);

    out->valid_mask   = valid_mask;
    out->outlier_mask = outlier_mask;
    out->timestamp_us = in->timestamp_us;
    out->reserved[0]  = 0U;
    out->reserved[1]  = 0U;

    /* Track worst delta across all axes */
    int16_t worst = d_ax;
    if (d_ay > worst) worst = d_ay;
    if (d_az > worst) worst = d_az;
    if (d_gx > worst) worst = d_gx;
    if (d_gy > worst) worst = d_gy;
    if (d_gz > worst) worst = d_gz;

    g_stats.last_check_us  = agent_time_now_us();
    g_stats.last_max_delta = worst;

    if (outlier_mask != 0U) {
        g_stats.outlier_count++;
        g_stats.divergence_count++;
        agent_log_event(AGENT_LOG_SENSOR_RECOVERED, outlier_mask, (uint32_t)worst);
        return AGENT_CROSSCHECK_DIVERGED;
    }

    return AGENT_CROSSCHECK_OK;
}

/* ---------- STATS ---------- */
const agent_crosscheck_stats_t *agent_crosscheck_stats(void)
{
    return &g_stats;
}

/* ---------- IS OUTLIER ---------- */
bool agent_crosscheck_is_outlier(uint32_t mask, uint32_t channel)
{
    return (mask & (1UL << channel)) != 0U;
}