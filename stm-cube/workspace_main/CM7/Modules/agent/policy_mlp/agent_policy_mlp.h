#ifndef AGENT_POLICY_MLP_H
#define AGENT_POLICY_MLP_H

#include "../core/agent_types.h"
#include "../policy/agent_policy_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY MLP — FROZEN
 * ============================================================
 * Tiny int8 MLP policy: 16 → 32 → 16 → 12 (softmax).
 * Registered as a policy in the registry.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 200 µs max per inference.
 * ============================================================ */

#define AGENT_POLICY_MLP_INPUT_DIM      16U
#define AGENT_POLICY_MLP_HIDDEN1_DIM    32U
#define AGENT_POLICY_MLP_HIDDEN2_DIM    16U
#define AGENT_POLICY_MLP_OUTPUT_DIM     AGENT_POLICY_ID_COUNT

/* ---------- Context ---------- */
typedef struct {
    float                   input[AGENT_POLICY_MLP_INPUT_DIM];
    float                   hidden1[AGENT_POLICY_MLP_HIDDEN1_DIM];
    float                   hidden2[AGENT_POLICY_MLP_HIDDEN2_DIM];
    float                   output[AGENT_POLICY_MLP_OUTPUT_DIM];
    uint32_t                argmax_idx;
    float                   argmax_value;
    uint32_t                inference_us;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_policy_mlp_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_policy_mlp_init(void);
agent_status_t              agent_policy_mlp_reset(void);

agent_status_t              agent_policy_mlp_set_input(uint32_t idx, float value);
agent_status_t              agent_policy_mlp_clear_input(void);

agent_status_t              agent_policy_mlp_infer(void);

uint32_t                    agent_policy_mlp_argmax(void);
float                       agent_policy_mlp_prob(uint32_t idx);

const agent_policy_mlp_ctx_t *agent_policy_mlp_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_MLP_H */