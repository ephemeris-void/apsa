#ifndef AGENT_POLICY_TF_SAFETY_H
#define AGENT_POLICY_TF_SAFETY_H

#include "agent_policy_tf.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY TF SAFETY — FROZEN
 * ============================================================
 * Envelope gate + output validation for transformer policy.
 * Blocks TF when envelope violated, confidence low, or EKF diverged.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per check.
 * ============================================================ */

agent_status_t  agent_policy_tf_safety_init(void);
agent_status_t  agent_policy_tf_safety_reset(void);

bool            agent_policy_tf_safety_check(void);

bool            agent_policy_tf_safety_allow_output(uint32_t action_idx);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_TF_SAFETY_H */