#include "agent_policy_mlp.h"
#include "agent_policy_mlp_model.h"
#include "agent_policy_mlp_infer.h"
#include "agent_policy_mlp_quant.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY MLP — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 200 µs per inference
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_mlp_ctx_t g_mlp;
static bool                              g_mlp_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_mlp_init(void)
{
    memset(&g_mlp, 0, sizeof(g_mlp));    /* zero-init incl. reserved[] */

    g_mlp.argmax_idx   = 0U;
    g_mlp.argmax_value = 0.0f;
    g_mlp.inference_us = 0U;

    agent_policy_mlp_model_init();
    agent_policy_mlp_infer_init();

    g_mlp_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_mlp_reset(void)
{
    g_mlp_initialized = false;
    return agent_policy_mlp_init();
}

/* ---------- SET INPUT ---------- */
agent_status_t agent_policy_mlp_set_input(uint32_t idx, float value)
{
    if (!g_mlp_initialized)                    return AGENT_ERR_NOT_INIT;
    if (idx >= AGENT_POLICY_MLP_INPUT_DIM)     return AGENT_ERR;

    g_mlp.input[idx] = value;
    return AGENT_OK;
}

/* ---------- CLEAR INPUT ---------- */
agent_status_t agent_policy_mlp_clear_input(void)
{
    if (!g_mlp_initialized) return AGENT_ERR_NOT_INIT;

    for (uint32_t i = 0U; i < AGENT_POLICY_MLP_INPUT_DIM; i++) {
        g_mlp.input[i] = 0.0f;
    }
    return AGENT_OK;
}

/* ---------- INFER ---------- */
agent_status_t agent_policy_mlp_infer(void)
{
    if (!g_mlp_initialized) return AGENT_ERR_NOT_INIT;

    uint32_t t0 = agent_time_now_us();

    agent_status_t st = agent_policy_mlp_infer_run(g_mlp.input,
                                                   g_mlp.hidden1,
                                                   g_mlp.hidden2,
                                                   g_mlp.output);

    uint32_t t1 = agent_time_now_us();
    g_mlp.inference_us = t1 - t0;

    if (st != AGENT_OK) {
        agent_log_event(AGENT_LOG_AI_FAIL, 0xC001U, g_mlp.inference_us);
        return st;
    }

    /* Argmax over output classes */
    uint32_t best   = 0U;
    float    best_v = g_mlp.output[0];

    for (uint32_t i = 1U; i < AGENT_POLICY_MLP_OUTPUT_DIM; i++) {
        if (g_mlp.output[i] > best_v) {
            best_v = g_mlp.output[i];
            best   = i;
        }
    }

    g_mlp.argmax_idx   = best;
    g_mlp.argmax_value = best_v;

    if (g_mlp.inference_us > 5000U) {
        agent_log_event(AGENT_LOG_WCET_VIOLATE, 0xC002U, g_mlp.inference_us);
    }

    return AGENT_OK;
}

/* ---------- QUERIES ---------- */
uint32_t agent_policy_mlp_argmax(void)
{
    return g_mlp.argmax_idx;
}

float agent_policy_mlp_prob(uint32_t idx)
{
    if (idx >= AGENT_POLICY_MLP_OUTPUT_DIM) {
        return 0.0f;
    }
    return g_mlp.output[idx];
}

const agent_policy_mlp_ctx_t *agent_policy_mlp_ctx(void)
{
    return &g_mlp;
}