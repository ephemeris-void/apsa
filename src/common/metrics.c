/* ============================================================
 * METRICS — IMPLEMENTATION
 * ============================================================ */

#include "metrics.h"
#include "../hal/hal_time.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <stdio.h>

/* ---------- Internal State ---------- */
static metrics_block_t g_metrics;

/* ---------- Local timing start cycles ---------- */
static uint32_t g_start_cycles[METRIC_COUNT];

/* EWMA smoothing factor: 1/8 */
#define METRIC_EWMA_SHIFT   3U

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t metrics_init(void)
{
    for (uint32_t i = 0U; i < METRIC_COUNT; i++) {
        metric_slot_t *s = &g_metrics.slots[i];
        s->call_count    = 0U;
        s->error_count   = 0U;
        s->wcet_us       = 0U;
        s->last_us       = 0U;
        s->total_us      = 0U;
        s->min_us        = 0xFFFFFFFFU;
        s->ewma_us       = 0U;
        s->deadline_miss = 0U;
    }

    g_metrics.system_uptime_ms   = 0U;
    g_metrics.system_loop_count  = 0U;
    g_metrics.system_wcet_max_us = 0U;
    g_metrics.initialized        = 1U;

    for (uint32_t i = 0U; i < METRIC_COUNT; i++) {
        g_start_cycles[i] = 0U;
    }

    return PROJ_OK;
}

/* ============================================================
 * BEGIN — Record start cycle
 * ============================================================ */
void metrics_begin(metric_id_t id)
{
    if (id >= METRIC_COUNT) {
        return;
    }
    g_start_cycles[id] = hal_time_cycles();
}

/* ============================================================
 * END — Compute elapsed, update stats
 * ============================================================ */
static void metrics_finish(metric_id_t id, bool is_error)
{
    if (id >= METRIC_COUNT) {
        return;
    }

    uint32_t end_cycles = hal_time_cycles();
    uint32_t elapsed_cycles = end_cycles - g_start_cycles[id];
    uint32_t elapsed_us = hal_time_cycles_to_us(elapsed_cycles);

    metric_slot_t *s = &g_metrics.slots[id];

    s->call_count++;
    if (is_error) {
        s->error_count++;
    }

    s->last_us  = elapsed_us;
    s->total_us += elapsed_us;

    if (elapsed_us > s->wcet_us) {
        s->wcet_us = elapsed_us;
    }

    if (elapsed_us < s->min_us) {
        s->min_us = elapsed_us;
    }

    /* EWMA update */
    if (s->ewma_us == 0U) {
        s->ewma_us = elapsed_us;
    } else {
        /* ewma = ewma + (sample - ewma) / 8 */
        int32_t delta = (int32_t)elapsed_us - (int32_t)s->ewma_us;
        s->ewma_us = (uint32_t)((int32_t)s->ewma_us +
                                (delta >> METRIC_EWMA_SHIFT));
    }
}

void metrics_end(metric_id_t id)
{
    metrics_finish(id, false);
}

void metrics_end_error(metric_id_t id)
{
    metrics_finish(id, true);
}

/* ============================================================
 * QUERY
 * ============================================================ */
const metric_slot_t *metrics_get(metric_id_t id)
{
    if (id >= METRIC_COUNT) {
        return NULL;
    }
    return &g_metrics.slots[id];
}

const metrics_block_t *metrics_block(void)
{
    return &g_metrics;
}

/* ============================================================
 * RESET
 * ============================================================ */
void metrics_reset(metric_id_t id)
{
    if (id >= METRIC_COUNT) {
        return;
    }

    metric_slot_t *s = &g_metrics.slots[id];
    s->call_count    = 0U;
    s->error_count   = 0U;
    s->wcet_us       = 0U;
    s->last_us       = 0U;
    s->total_us      = 0U;
    s->min_us        = 0xFFFFFFFFU;
    s->ewma_us       = 0U;
    s->deadline_miss = 0U;
}

void metrics_reset_all(void)
{
    for (uint32_t i = 0U; i < METRIC_COUNT; i++) {
        metrics_reset((metric_id_t)i);
    }
    g_metrics.system_uptime_ms   = 0U;
    g_metrics.system_loop_count  = 0U;
    g_metrics.system_wcet_max_us = 0U;
}

/* ============================================================
 * SYSTEM COUNTERS
 * ============================================================ */
void metrics_inc_loop(void)
{
    g_metrics.system_loop_count++;
}

void metrics_set_wcet(uint32_t us)
{
    if (us > g_metrics.system_wcet_max_us) {
        g_metrics.system_wcet_max_us = us;
    }
}

void metrics_set_uptime(uint32_t ms)
{
    g_metrics.system_uptime_ms = ms;
}

/* ============================================================
 * DUMP
 * ============================================================ */
static const char *metric_name(metric_id_t id)
{
    switch (id) {
    case METRIC_SENSOR_READ:    return "SENSOR_READ";
    case METRIC_AGENT_STEP:     return "AGENT_STEP";
    case METRIC_EKF_UPDATE:     return "EKF_UPDATE";
    case METRIC_AI_INFERENCE:   return "AI_INFERENCE";
    case METRIC_CONTROL_UPDATE: return "CONTROL_UPDATE";
    case METRIC_ROS_PUBLISH:    return "ROS_PUBLISH";
    case METRIC_IPC_SEND:       return "IPC_SEND";
    case METRIC_IPC_RECV:       return "IPC_RECV";
    default:                    return "UNKNOWN";
    }
}

void metrics_dump(void)
{
    printk("\n=== METRICS DUMP ===\n");
    printk("Uptime      : %u ms\n", (unsigned)g_metrics.system_uptime_ms);
    printk("Loops       : %u\n", (unsigned)g_metrics.system_loop_count);
    printk("WCET (max)  : %u us\n", (unsigned)g_metrics.system_wcet_max_us);
    printk("\n%-16s %10s %10s %10s %10s %10s %10s\n",
           "Metric", "Calls", "Errors", "Min_us", "Avg_us", "Max_us", "EWMA_us");

    for (uint32_t i = 0U; i < METRIC_COUNT; i++) {
        const metric_slot_t *s = &g_metrics.slots[i];
        if (s->call_count == 0U) {
            continue;
        }
        uint32_t avg = (s->call_count > 0U) ? (s->total_us / s->call_count) : 0U;
        printk("%-16s %10u %10u %10u %10u %10u %10u\n",
               metric_name((metric_id_t)i),
               (unsigned)s->call_count,
               (unsigned)s->error_count,
               (unsigned)s->min_us,
               (unsigned)avg,
               (unsigned)s->wcet_us,
               (unsigned)s->ewma_us);
    }
    printk("=== END METRICS ===\n\n");
}