#ifndef AGENT_POLICY_TF_INFER_H
#define AGENT_POLICY_TF_INFER_H

#include "agent_policy_tf.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY TF INFERENCE — FROZEN
 * ============================================================
 * Full forward pass:
 *   tokenize → QKV → attention → FF → pool → output.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5000 µs max per run.
 * ============================================================ */

agent_status_t  agent_policy_tf_infer_init(void);

agent_status_t  agent_policy_tf_infer_run(agent_policy_tf_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_TF_INFER_H */