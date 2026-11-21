#ifndef AGENT_POLICY_MLP_INFER_H
#define AGENT_POLICY_MLP_INFER_H

#include "agent_policy_mlp.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY MLP INFERENCE — FROZEN
 * ============================================================
 * int8 MLP forward pass: 16 → 32 → 16 → 12.
 * ReLU hidden, linear output.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 200 µs max per run.
 * ============================================================ */

agent_status_t  agent_policy_mlp_infer_init(void);

agent_status_t  agent_policy_mlp_infer_run(const float *input,
                                           float       *hidden1,
                                           float       *hidden2,
                                           float       *output);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_MLP_INFER_H */