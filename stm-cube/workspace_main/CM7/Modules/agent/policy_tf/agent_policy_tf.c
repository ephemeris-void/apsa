#include "agent_policy_tf.h"
#include "agent_policy_tf_model.h"
#include "agent_policy_tf_infer.h"
#include "agent_policy_tf_tokenizer.h"
#include "agent_policy_tf_softmax.h"
#include "agent_policy_tf_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY TF — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5000 µs per inference
 *
 * SAFETY: Envelope checked before + after inference.
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_tf_ctx_t g_tf;
static bool                             g_tf_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_tf_init(void)
{
    memset(&g_tf, 0, sizeof(g_tf));    /* zero-init incl. reserved[] */

    g_tf.argmax_idx   = 0U;
    g_tf.argmax_value = 0.0f;
    g_tf.inference_us = 0U;

    agent_policy_tf_model_init();
    agent_policy_tf_infer_init();
    agent_policy_tf_tokenizer_init();
    agent_policy_tf_softmax_init();
    agent_policy_tf_safety_init();

    g_tf_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_tf_reset(void)
{
    g_tf_initialized = false;
    return agent_policy_tf_init();
}

/* ---------- SET TOKEN ---------- */
agent_status_t agent_policy_tf_set_token(uint32_t t, uint32_t f, float value)
{
    if (!g_tf_initialized)          return AGENT_ERR_NOT_INIT;
    if (t >= AGENT_TF_SEQ_LEN)      return AGENT_ERR;
    if (f >= AGENT_TF_FEATURE_DIM)  return AGENT_ERR;

    return agent_policy_tf_tokenizer_set(t, f, value);
}

/* ---------- CLEAR TOKENS ---------- */
agent_status_t agent_policy_tf_clear_tokens(void)
{
    if (!g_tf_initialized) return AGENT_ERR_NOT_INIT;
    return agent_policy_tf_tokenizer_clear();
}

/* ---------- INFER ---------- */
agent_status_t agent_policy_tf_infer(void)
{
    if (!g_tf_initialized) return AGENT_ERR_NOT_INIT;

    /* SAFETY: pre-inference envelope gate */
    if (!agent_policy_tf_safety_check()) {
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0xD001U, 0U);
        return AGENT_ERR;
    }

    uint32_t t0 = agent_time_now_us();

    agent_status_t st = agent_policy_tf_infer_run(&g_tf);

    uint32_t t1 = agent_time_now_us();
    g_tf.inference_us = t1 - t0;

    if (st != AGENT_OK) {
        agent_log_event(AGENT_LOG_AI_FAIL, 0xD002U, g_tf.inference_us);
        return st;
    }

    /* Argmax over output classes */
    uint32_t best   = 0U;
    float    best_v = g_tf.output[0];

    for (uint32_t i = 1U; i < AGENT_TF_OUTPUT_DIM; i++) {
        if (g_tf.output[i] > best_v) {
            best_v = g_tf.output[i];
            best   = i;
        }
    }

    g_tf.argmax_idx   = best;
    g_tf.argmax_value = best_v;

    if (g_tf.inference_us > 50000U) {
        agent_log_event(AGENT_LOG_WCET_VIOLATE, 0xD003U, g_tf.inference_us);
    }

    return AGENT_OK;
}

/* ---------- QUERIES ---------- */
uint32_t agent_policy_tf_argmax(void)
{
    return g_tf.argmax_idx;
}

float agent_policy_tf_prob(uint32_t idx)
{
    if (idx >= AGENT_TF_OUTPUT_DIM) {
        return 0.0f;
    }
    return g_tf.output[idx];
}

const agent_policy_tf_ctx_t *agent_policy_tf_ctx(void)
{
    return &g_tf;
}