#ifndef AGENT_POLICY_MLP_MODEL_H
#define AGENT_POLICY_MLP_MODEL_H

#include "agent_policy_mlp.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY MLP MODEL — FROZEN
 * ============================================================
 * int8 quantized weights + per-layer scales/zero-points.
 * Weights linked at build time; can be swapped via model manager.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * ============================================================ */

typedef struct {
    const int8_t  *w1;        /* 16 × 32 */
    const int32_t *b1;        /* 32 */
    const int8_t  *w2;        /* 32 × 16 */
    const int32_t *b2;        /* 16 */
    const int8_t  *w3;        /* 16 × 12 */
    const int32_t *b3;        /* 12 */
    float          scale_in;
    float          scale_h1;
    float          scale_h2;
    float          scale_out;
    int32_t        zp_in;
    int32_t        zp_h1;
    int32_t        zp_h2;
    int32_t        zp_out;
    uint32_t       reserved[2];   /* ABI-safe future extension */
} agent_policy_mlp_weights_t;

agent_status_t                     agent_policy_mlp_model_init(void);
const agent_policy_mlp_weights_t  *agent_policy_mlp_model_weights(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_MLP_MODEL_H */