#include "agent_stats.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"

/* ============================================================
 * AGENT STATS — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per call
 * ============================================================ */

static AGENT_DTCM agent_stats_t g_stats;
static bool                     g_stats_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_stats_init(void)
{
    agent_stats_reset();
    g_stats_initialized = true;
    return AGENT_OK;
}

/* ---------- RESET ---------- */
void agent_stats_reset(void)
{
    uint32_t *p = (uint32_t *)&g_stats;
    uint32_t  n = sizeof(g_stats) / sizeof(uint32_t);

    for (uint32_t i = 0U; i < n; i++) {
        p[i] = 0U;
    }

    g_stats.ai_latency_min_us = 0xFFFFFFFFU;
}

/* ---------- SNAPSHOT ---------- */
void agent_stats_snapshot(agent_stats_t *out)
{
    if (out == NULL) {
        return;
    }

    agent_context_full_t *ctx = agent_context_get();

    g_stats.iterations          = ctx->iteration_count;
    g_stats.faults_total        = ctx->fault_count;
    g_stats.recoveries_total    = ctx->recovery_count;
    g_stats.deadline_miss_total = ctx->deadline_miss_count;
    g_stats.wcet_violate_total  = ctx->wcet_violate_count;
    g_stats.ai_inference_total  = ctx->ai_inference_total;
    g_stats.imu_samples_total   = ctx->imu_samples_total;
    g_stats.imu_dropouts_total  = ctx->imu_samples_dropped;

    *out = g_stats;
}

/* ---------- INCREMENTS ---------- */
void agent_stats_inc_iteration(void)            { g_stats.iterations++; }
void agent_stats_inc_transition_ok(void)        { g_stats.transitions_ok++; }
void agent_stats_inc_transition_blocked(void)   { g_stats.transitions_blocked++; }
void agent_stats_inc_transition_forced(void)    { g_stats.transitions_forced++; }
void agent_stats_inc_fault(void)                { g_stats.faults_total++; }
void agent_stats_inc_recovery(void)             { g_stats.recoveries_total++; }
void agent_stats_inc_deadline_miss(void)        { g_stats.deadline_miss_total++; }
void agent_stats_inc_wcet_violate(void)         { g_stats.wcet_violate_total++; }
void agent_stats_inc_imu_sample(void)           { g_stats.imu_samples_total++; }
void agent_stats_inc_imu_dropout(void)          { g_stats.imu_dropouts_total++; }

void agent_stats_inc_ai_inference(uint32_t latency_us)
{
    g_stats.ai_inference_total++;

    if (latency_us < g_stats.ai_latency_min_us) {
        g_stats.ai_latency_min_us = latency_us;
    }
    if (latency_us > g_stats.ai_latency_max_us) {
        g_stats.ai_latency_max_us = latency_us;
    }

    uint32_t n = g_stats.ai_inference_total;
    g_stats.ai_latency_avg_us =
        (uint32_t)(((uint64_t)g_stats.ai_latency_avg_us * (n - 1U) + latency_us) / n);
}

/* ---------- STATE ENTRY COUNTER ---------- */
void agent_stats_inc_state_entry(agent_state_t state)
{
    if (state < AGENT_STATE_MAX) {
        g_stats.state_entries[state]++;
    }
}