#include "agent_policy_mlp_model.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY MLP MODEL — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Weights in Flash (const) — zero RAM cost
 * ============================================================ */

/* ---------- Weight storage (placeholder, populated at integration) ---------- */
static const int8_t  g_w1[AGENT_POLICY_MLP_INPUT_DIM   * AGENT_POLICY_MLP_HIDDEN1_DIM];
static const int32_t g_b1[AGENT_POLICY_MLP_HIDDEN1_DIM];
static const int8_t  g_w2[AGENT_POLICY_MLP_HIDDEN1_DIM * AGENT_POLICY_MLP_HIDDEN2_DIM];
static const int32_t g_b2[AGENT_POLICY_MLP_HIDDEN2_DIM];
static const int8_t  g_w3[AGENT_POLICY_MLP_HIDDEN2_DIM * AGENT_POLICY_MLP_OUTPUT_DIM];
static const int32_t g_b3[AGENT_POLICY_MLP_OUTPUT_DIM];

/* PERF: weights descriptor in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_mlp_weights_t g_weights;
static bool                                  g_model_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_mlp_model_init(void)
{
    memset(&g_weights, 0, sizeof(g_weights));   /* zero-init incl. reserved[] */

    g_weights.w1        = g_w1;
    g_weights.b1        = g_b1;
    g_weights.w2        = g_w2;
    g_weights.b2        = g_b2;
    g_weights.w3        = g_w3;
    g_weights.b3        = g_b3;

    g_weights.scale_in  = 1.0f;
    g_weights.scale_h1  = 1.0f;
    g_weights.scale_h2  = 1.0f;
    g_weights.scale_out = 1.0f;

    g_weights.zp_in     = 0;
    g_weights.zp_h1     = 0;
    g_weights.zp_h2     = 0;
    g_weights.zp_out    = 0;

    g_model_initialized = true;
    return AGENT_OK;
}

/* ---------- WEIGHTS ---------- */
const agent_policy_mlp_weights_t *agent_policy_mlp_model_weights(void)
{
    return &g_weights;
}