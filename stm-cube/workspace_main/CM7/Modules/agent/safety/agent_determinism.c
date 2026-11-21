#include "agent_determinism.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT DETERMINISM — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per check
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_det_state_t g_det;
static bool                         g_det_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_determinism_init(void)
{
    memset(&g_det, 0, sizeof(g_det));    /* zero-init incl. reserved[] */

    g_det.head           = 0U;
    g_det.count          = 0U;
    g_det.check_count    = 0U;
    g_det.mismatch_count = 0U;
    g_det.last_check_us  = 0U;
    g_det.enabled        = true;

    g_det_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_determinism_reset(void)
{
    g_det_initialized = false;
    return agent_determinism_init();
}

/* ---------- ENABLE ---------- */
agent_status_t agent_determinism_enable(bool en)
{
    g_det.enabled = en;
    return AGENT_OK;
}

/* ---------- HASH (FNV-1a) ---------- */
uint32_t agent_determinism_hash(uint32_t state,
                                uint32_t priority,
                                uint32_t action_id,
                                uint32_t iteration)
{
    uint32_t h = 2166136261U;

    h ^= state;      h *= 16777619U;
    h ^= priority;   h *= 16777619U;
    h ^= action_id;  h *= 16777619U;
    h ^= iteration;  h *= 16777619U;

    return h;
}

/* ---------- RECORD ---------- */
agent_status_t agent_determinism_record(void)
{
    if (!g_det_initialized) return AGENT_ERR_NOT_INIT;
    if (!g_det.enabled)     return AGENT_OK;

    agent_context_full_t *ctx = agent_context_get();

    uint32_t idx = g_det.head;
    agent_det_sample_t *s = &g_det.history[idx];

    s->state        = (uint32_t)ctx->current_state;
    s->priority     = (uint32_t)ctx->current_priority;
    s->action_id    = 0U;
    s->iteration    = ctx->iteration_count;
    s->timestamp_us = agent_time_now_us();

    s->hash = agent_determinism_hash(s->state,
                                     s->priority,
                                     s->action_id,
                                     s->iteration);

    g_det.head = (idx + 1U) % AGENT_DET_HISTORY_DEPTH;
    if (g_det.count < AGENT_DET_HISTORY_DEPTH) {
        g_det.count++;
    }

    return AGENT_OK;
}

/* ---------- CHECK ---------- */
agent_det_result_t agent_determinism_check(uint32_t iteration)
{
    if (!g_det_initialized) return AGENT_DET_RESULT_FAIL;
    if (!g_det.enabled)     return AGENT_DET_RESULT_DISABLED;

    uint32_t now = agent_time_now_us();
    g_det.last_check_us = now;
    g_det.check_count++;

    /* Look for matching iteration in history */
    for (uint32_t i = 0U; i < g_det.count; i++) {

        const agent_det_sample_t *s = &g_det.history[i];

        if (s->iteration != iteration) {
            continue;
        }

        uint32_t h = agent_determinism_hash(s->state,
                                            s->priority,
                                            s->action_id,
                                            s->iteration);

        if (h != s->hash) {
            g_det.mismatch_count++;
            agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0xE001U, iteration);
            return AGENT_DET_RESULT_MISMATCH;
        }

        return AGENT_DET_RESULT_OK;
    }

    return AGENT_DET_RESULT_OK;
}

/* ---------- CLEAR ---------- */
void agent_determinism_clear(void)
{
    g_det.head  = 0U;
    g_det.count = 0U;
}

/* ---------- STATE ---------- */
const agent_det_state_t *agent_determinism_state(void)
{
    return &g_det;
}