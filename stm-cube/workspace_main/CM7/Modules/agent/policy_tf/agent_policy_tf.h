#ifndef AGENT_POLICY_TF_H
#define AGENT_POLICY_TF_H

#include "../core/agent_types.h"
#include "../policy/agent_policy_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY TF — FROZEN
 * ============================================================
 * Tiny distilled transformer policy.
 * Seq (16) × Features (8) → embed (16) → attention → 12 classes.
 * SAFETY: output envelope-checked. Frozen by default.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5000 µs max per inference.
 * ============================================================ */

#define AGENT_TF_SEQ_LEN            16U
#define AGENT_TF_FEATURE_DIM        8U
#define AGENT_TF_EMBED_DIM          16U
#define AGENT_TF_NUM_HEADS          2U
#define AGENT_TF_HEAD_DIM           (AGENT_TF_EMBED_DIM / AGENT_TF_NUM_HEADS)
#define AGENT_TF_FF_DIM             32U
#define AGENT_TF_NUM_LAYERS         1U
#define AGENT_TF_OUTPUT_DIM         AGENT_POLICY_ID_COUNT

/* ---------- Context ---------- */
typedef struct {
    float    tokens[AGENT_TF_SEQ_LEN][AGENT_TF_EMBED_DIM];
    float    q[AGENT_TF_SEQ_LEN][AGENT_TF_EMBED_DIM];
    float    k[AGENT_TF_SEQ_LEN][AGENT_TF_EMBED_DIM];
    float    v[AGENT_TF_SEQ_LEN][AGENT_TF_EMBED_DIM];
    float    attn[AGENT_TF_SEQ_LEN][AGENT_TF_SEQ_LEN];
    float    ff[AGENT_TF_SEQ_LEN][AGENT_TF_FF_DIM];
    float    pooled[AGENT_TF_EMBED_DIM];
    float    output[AGENT_TF_OUTPUT_DIM];
    uint32_t argmax_idx;
    float    argmax_value;
    uint32_t inference_us;
    uint32_t reserved[2];   /* ABI-safe future extension */
} agent_policy_tf_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_policy_tf_init(void);
agent_status_t              agent_policy_tf_reset(void);

agent_status_t              agent_policy_tf_set_token(uint32_t t,
                                                      uint32_t f,
                                                      float    value);
agent_status_t              agent_policy_tf_clear_tokens(void);

agent_status_t              agent_policy_tf_infer(void);

uint32_t                    agent_policy_tf_argmax(void);
float                       agent_policy_tf_prob(uint32_t idx);

const agent_policy_tf_ctx_t *agent_policy_tf_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_TF_H */