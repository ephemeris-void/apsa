#include "agent_policy_tf_model.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY TF MODEL — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Weights in Flash (const) — zero RAM cost
 * ============================================================ */

/* ---------- Weight storage (placeholder, populated at integration) ---------- */
static const int8_t g_wq [AGENT_TF_EMBED_DIM * AGENT_TF_EMBED_DIM];
static const int8_t g_wk [AGENT_TF_EMBED_DIM * AGENT_TF_EMBED_DIM];
static const int8_t g_wv [AGENT_TF_EMBED_DIM * AGENT_TF_EMBED_DIM];
static const int8_t g_wo [AGENT_TF_EMBED_DIM * AGENT_TF_EMBED_DIM];
static const int8_t g_w1 [AGENT_TF_EMBED_DIM * AGENT_TF_FF_DIM];
static const int8_t g_w2 [AGENT_TF_FF_DIM    * AGENT_TF_EMBED_DIM];
static const int8_t g_wout[AGENT_TF_EMBED_DIM * AGENT_TF_OUTPUT_DIM];

/* PERF: weights descriptor in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_tf_weights_t g_tf_weights;
static bool                                  g_tf_model_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_tf_model_init(void)
{
    memset(&g_tf_weights, 0, sizeof(g_tf_weights));   /* zero-init incl. reserved[] */

    g_tf_weights.wq        = g_wq;
    g_tf_weights.wk        = g_wk;
    g_tf_weights.wv        = g_wv;
    g_tf_weights.wo        = g_wo;
    g_tf_weights.w1        = g_w1;
    g_tf_weights.w2        = g_w2;
    g_tf_weights.wout      = g_wout;

    g_tf_weights.scale_q   = 1.0f;
    g_tf_weights.scale_k   = 1.0f;
    g_tf_weights.scale_v   = 1.0f;
    g_tf_weights.scale_o   = 1.0f;
    g_tf_weights.scale_ff1 = 1.0f;
    g_tf_weights.scale_ff2 = 1.0f;
    g_tf_weights.scale_out = 1.0f;

    g_tf_model_initialized = true;
    return AGENT_OK;
}

/* ---------- WEIGHTS ---------- */
const agent_policy_tf_weights_t *agent_policy_tf_model_weights(void)
{
    return &g_tf_weights;
}