#include "agent_policy_mlp_infer.h"
#include "agent_policy_mlp_model.h"
#include "agent_policy_mlp_quant.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * AGENT POLICY MLP INFERENCE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 200 µs per run
 * ============================================================ */

static bool g_infer_initialized = false;

/* ---------- ReLU ---------- */
AGENT_INLINE_FAST float relu_f(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

/* ---------- INIT ---------- */
agent_status_t agent_policy_mlp_infer_init(void)
{
    g_infer_initialized = true;
    return AGENT_OK;
}

/* ---------- FORWARD PASS ---------- */
agent_status_t agent_policy_mlp_infer_run(const float *input,
                                          float       *hidden1,
                                          float       *hidden2,
                                          float       *output)
{
    if (input == NULL || hidden1 == NULL || hidden2 == NULL || output == NULL) {
        return AGENT_ERR;
    }

    if (!g_infer_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    const agent_policy_mlp_weights_t *w = agent_policy_mlp_model_weights();

    /* Clear outputs */
    memset(hidden1, 0, sizeof(float) * AGENT_POLICY_MLP_HIDDEN1_DIM);
    memset(hidden2, 0, sizeof(float) * AGENT_POLICY_MLP_HIDDEN2_DIM);
    memset(output,  0, sizeof(float) * AGENT_POLICY_MLP_OUTPUT_DIM);

    /* Layer 1: input → hidden1 (16 × 32) */
    for (uint32_t j = 0U; j < AGENT_POLICY_MLP_HIDDEN1_DIM; j++) {

        float sum = 0.0f;

        for (uint32_t i = 0U; i < AGENT_POLICY_MLP_INPUT_DIM; i++) {
            int8_t wi = w->w1[i * AGENT_POLICY_MLP_HIDDEN1_DIM + j];
            sum += input[i] * (float)wi * w->scale_in;
        }

        sum += (float)w->b1[j] * w->scale_in;
        hidden1[j] = relu_f(sum);
    }

    /* Layer 2: hidden1 → hidden2 (32 × 16) */
    for (uint32_t j = 0U; j < AGENT_POLICY_MLP_HIDDEN2_DIM; j++) {

        float sum = 0.0f;

        for (uint32_t i = 0U; i < AGENT_POLICY_MLP_HIDDEN1_DIM; i++) {
            int8_t wi = w->w2[i * AGENT_POLICY_MLP_HIDDEN2_DIM + j];
            sum += hidden1[i] * (float)wi * w->scale_h1;
        }

        sum += (float)w->b2[j] * w->scale_h1;
        hidden2[j] = relu_f(sum);
    }

    /* Layer 3: hidden2 → output (16 × 12) */
    for (uint32_t j = 0U; j < AGENT_POLICY_MLP_OUTPUT_DIM; j++) {

        float sum = 0.0f;

        for (uint32_t i = 0U; i < AGENT_POLICY_MLP_HIDDEN2_DIM; i++) {
            int8_t wi = w->w3[i * AGENT_POLICY_MLP_OUTPUT_DIM + j];
            sum += hidden2[i] * (float)wi * w->scale_h2;
        }

        sum += (float)w->b3[j] * w->scale_h2;
        output[j] = sum;
    }

    return AGENT_OK;
}