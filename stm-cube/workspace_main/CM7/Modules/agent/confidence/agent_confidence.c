#include "agent_confidence.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT CONFIDENCE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per update
 *
 * SINGLE SOURCE OF TRUTH: g_state.current
 * Context->confidence is mirrored for display/debug.
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_confidence_state_t g_state;
static AGENT_DTCM agent_confidence_input_t g_input;

/* ---------- Clamp ---------- */
AGENT_INLINE_FAST float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

/* ============================================================
 * INIT / RESET
 * ============================================================ */

agent_status_t agent_confidence_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    memset(&g_input, 0, sizeof(g_input));

    for (uint32_t i = 0U; i < AGENT_CONFIDENCE_SRC_COUNT; i++) {
        g_input.weight[i] = 1.0f;
        g_input.value[i]  = 0.0f;
        g_input.valid[i]  = false;
    }

    g_state.current        = 1.0f;
    g_state.min_ever       = 1.0f;
    g_state.max_ever       = 1.0f;
    g_state.update_count   = 0U;
    g_state.last_update_us = agent_time_now_us();

    /* Mirror to context */
    agent_context_full_t *ctx = agent_context_get();
    ctx->confidence = 1.0f;

    return AGENT_OK;
}

agent_status_t agent_confidence_reset(void)
{
    return agent_confidence_init();
}

/* ============================================================
 * PUBLIC API — single-value set/get
 * ============================================================
 * Used by agent_states (fusion, recovery, landing) to bump
 * confidence after IMU recovery.
 * ============================================================ */

void agent_confidence_set(float c)
{
    c = clamp01(c);

    g_state.current = c;

    if (c < g_state.min_ever) g_state.min_ever = c;
    if (c > g_state.max_ever) g_state.max_ever = c;

    /* Mirror to context for display/debug */
    agent_context_full_t *ctx = agent_context_get();
    ctx->confidence = c;
}

float agent_confidence_get(void)
{
    return g_state.current;
}

/* ============================================================
 * PER-SOURCE MANAGEMENT
 * ============================================================ */

void agent_confidence_set_source(agent_confidence_src_t src,
                                 float                  value,
                                 float                  weight)
{
    if (src >= AGENT_CONFIDENCE_SRC_COUNT) {
        return;
    }

    g_input.value[src]        = clamp01(value);
    g_input.weight[src]       = (weight < 0.0f) ? 0.0f : weight;
    g_input.valid[src]        = true;
    g_input.timestamp_us[src] = agent_time_now_us();
}

void agent_confidence_invalidate(agent_confidence_src_t src)
{
    if (src >= AGENT_CONFIDENCE_SRC_COUNT) {
        return;
    }
    g_input.valid[src] = false;
}

/* ============================================================
 * MULTI-SOURCE FUSION UPDATE
 * ============================================================
 * Weighted average of all valid sources, then low-pass smoothed.
 * Result stored in g_state.current, mirrored to context.
 * ============================================================ */

agent_status_t agent_confidence_update(const agent_confidence_input_t *in,
                                       agent_confidence_output_t      *out)
{
    if (in == NULL || out == NULL) {
        return AGENT_ERR;
    }

    float    wsum = 0.0f;
    float    vsum = 0.0f;
    float    vmin = 1.0f;
    float    vmax = 0.0f;
    uint32_t cnt  = 0U;

    for (uint32_t i = 0U; i < AGENT_CONFIDENCE_SRC_COUNT; i++) {

        if (!in->valid[i]) {
            continue;
        }

        float v = clamp01(in->value[i]);
        float w = (in->weight[i] < 0.0f) ? 0.0f : in->weight[i];

        wsum += w;
        vsum += v * w;

        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        cnt++;
    }

    float weighted = (wsum > 0.0f) ? (vsum / wsum) : 0.0f;

    /* Low-pass smoothing — prevents single-sample spikes */
    float alpha    = AGENT_CONFIDENCE_SMOOTH_ALPHA;
    float combined = (1.0f - alpha) * g_state.current + alpha * weighted;
    combined       = clamp01(combined);

    /* Write via public setter (single source of truth) */
    agent_confidence_set(combined);

    g_state.update_count++;
    g_state.last_update_us = agent_time_now_us();

    /* Prepare output */
    memset(out, 0, sizeof(*out));

    out->combined     = g_state.current;
    out->weighted     = weighted;
    out->min_value    = vmin;
    out->max_value    = vmax;
    out->valid_count  = cnt;
    out->timestamp_us = g_state.last_update_us;

    if (g_state.current < AGENT_CONFIDENCE_MIN) {
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                        (uint32_t)(g_state.current * 1000.0f),
                        cnt);
    }

    return AGENT_OK;
}

/* ============================================================
 * QUERIES
 * ============================================================ */

float agent_confidence_get_min(void)
{
    return g_state.min_ever;
}

float agent_confidence_get_max(void)
{
    return g_state.max_ever;
}

const agent_confidence_state_t *agent_confidence_state(void)
{
    return &g_state;
}