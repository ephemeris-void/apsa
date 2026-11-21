#ifndef AGENT_POLICY_TF_MODEL_H
#define AGENT_POLICY_TF_MODEL_H

#include "agent_policy_tf.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY TF MODEL — FROZEN
 * ============================================================
 * int8 weights for tiny transformer:
 *   - Q/K/V/O projections: 16 × 16
 *   - FF1: 16 × 32, FF2: 32 × 16
 *   - Output: 16 × 12
 * Weights in Flash (const). Swappable via model manager.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * ============================================================ */

typedef struct {
    const int8_t *wq;         /* 16 × 16 */
    const int8_t *wk;         /* 16 × 16 */
    const int8_t *wv;         /* 16 × 16 */
    const int8_t *wo;         /* 16 × 16 */
    const int8_t *w1;         /* 16 × 32 */
    const int8_t *w2;         /* 32 × 16 */
    const int8_t *wout;       /* 16 × 12 */
    float         scale_q;
    float         scale_k;
    float         scale_v;
    float         scale_o;
    float         scale_ff1;
    float         scale_ff2;
    float         scale_out;
    uint32_t      reserved[2];   /* ABI-safe future extension */
} agent_policy_tf_weights_t;

agent_status_t                  agent_policy_tf_model_init(void);
const agent_policy_tf_weights_t *agent_policy_tf_model_weights(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_TF_MODEL_H */