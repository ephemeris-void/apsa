#include "agent_policy_tf_infer.h"
#include "agent_policy_tf_model.h"
#include "agent_policy_tf_tokenizer.h"
#include "agent_policy_tf_softmax.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * AGENT POLICY TF INFERENCE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5000 µs per run
 * ============================================================ */

static bool g_tf_infer_initialized = false;

/* ---------- ReLU ---------- */
AGENT_INLINE_FAST float relu_f(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

/* ---------- Linear int8: y = x · W ---------- */
static void linear_i8(const float *in, uint32_t in_dim,
                      const int8_t *w, uint32_t out_dim,
                      float scale, float *out)
{
    for (uint32_t j = 0U; j < out_dim; j++) {

        float sum = 0.0f;

        for (uint32_t i = 0U; i < in_dim; i++) {
            int8_t wi = w[i * out_dim + j];
            sum += in[i] * (float)wi * scale;
        }

        out[j] = sum;
    }
}

/* ---------- INIT ---------- */
agent_status_t agent_policy_tf_infer_init(void)
{
    g_tf_infer_initialized = true;
    return AGENT_OK;
}

/* ---------- FORWARD PASS ---------- */
agent_status_t agent_policy_tf_infer_run(agent_policy_tf_ctx_t *ctx)
{
    if (ctx == NULL)              return AGENT_ERR;
    if (!g_tf_infer_initialized)  return AGENT_ERR_NOT_INIT;

    const agent_policy_tf_weights_t *w = agent_policy_tf_model_weights();

    /* 1. Tokenize + Embed */
    if (agent_policy_tf_tokenizer_embed(ctx->tokens) != AGENT_OK) {
        return AGENT_ERR;
    }

    /* 2. Q, K, V projections */
    for (uint32_t t = 0U; t < AGENT_TF_SEQ_LEN; t++) {
        linear_i8(ctx->tokens[t], AGENT_TF_EMBED_DIM,
                  w->wq, AGENT_TF_EMBED_DIM, w->scale_q, ctx->q[t]);
        linear_i8(ctx->tokens[t], AGENT_TF_EMBED_DIM,
                  w->wk, AGENT_TF_EMBED_DIM, w->scale_k, ctx->k[t]);
        linear_i8(ctx->tokens[t], AGENT_TF_EMBED_DIM,
                  w->wv, AGENT_TF_EMBED_DIM, w->scale_v, ctx->v[t]);
    }

    /* 3. Scaled dot-product attention scores */
    float inv_sqrt_d = 1.0f / sqrtf((float)AGENT_TF_HEAD_DIM);

    for (uint32_t i = 0U; i < AGENT_TF_SEQ_LEN; i++) {
        for (uint32_t j = 0U; j < AGENT_TF_SEQ_LEN; j++) {

            float dot = 0.0f;

            for (uint32_t d = 0U; d < AGENT_TF_EMBED_DIM; d++) {
                dot += ctx->q[i][d] * ctx->k[j][d];
            }

            ctx->attn[i][j] = dot * inv_sqrt_d;
        }
    }

    /* 4. Softmax over attention */
    agent_policy_tf_softmax_2d(ctx->attn, AGENT_TF_SEQ_LEN, AGENT_TF_SEQ_LEN);

    /* 5. Weighted sum: attn · V */
    for (uint32_t i = 0U; i < AGENT_TF_SEQ_LEN; i++) {
        for (uint32_t d = 0U; d < AGENT_TF_EMBED_DIM; d++) {

            float sum = 0.0f;

            for (uint32_t j = 0U; j < AGENT_TF_SEQ_LEN; j++) {
                sum += ctx->attn[i][j] * ctx->v[j][d];
            }

            ctx->tokens[i][d] = sum;
        }
    }

    /* 6. Output projection */
    for (uint32_t t = 0U; t < AGENT_TF_SEQ_LEN; t++) {
        linear_i8(ctx->tokens[t], AGENT_TF_EMBED_DIM,
                  w->wo, AGENT_TF_EMBED_DIM, w->scale_o, ctx->tokens[t]);
    }

    /* 7. Feed-forward (per token) */
    for (uint32_t t = 0U; t < AGENT_TF_SEQ_LEN; t++) {

        linear_i8(ctx->tokens[t], AGENT_TF_EMBED_DIM,
                  w->w1, AGENT_TF_FF_DIM, w->scale_ff1, ctx->ff[t]);

        for (uint32_t i = 0U; i < AGENT_TF_FF_DIM; i++) {
            ctx->ff[t][i] = relu_f(ctx->ff[t][i]);
        }

        linear_i8(ctx->ff[t], AGENT_TF_FF_DIM,
                  w->w2, AGENT_TF_EMBED_DIM, w->scale_ff2, ctx->tokens[t]);
    }

    /* 8. Pool (mean over sequence) */
    for (uint32_t d = 0U; d < AGENT_TF_EMBED_DIM; d++) {

        float sum = 0.0f;

        for (uint32_t t = 0U; t < AGENT_TF_SEQ_LEN; t++) {
            sum += ctx->tokens[t][d];
        }

        ctx->pooled[d] = sum / (float)AGENT_TF_SEQ_LEN;
    }

    /* 9. Output logits */
    linear_i8(ctx->pooled, AGENT_TF_EMBED_DIM,
              w->wout, AGENT_TF_OUTPUT_DIM, w->scale_out, ctx->output);

    return AGENT_OK;
}