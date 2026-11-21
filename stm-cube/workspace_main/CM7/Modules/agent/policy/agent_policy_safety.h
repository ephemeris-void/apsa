#ifndef AGENT_POLICY_SAFETY_H
#define AGENT_POLICY_SAFETY_H

#include "agent_policy.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY SAFETY — FROZEN
 * ============================================================
 * Validates policy candidates before arbitration.
 * Rejects candidates that violate safety invariants.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per call.
 * ============================================================ */

agent_status_t  agent_policy_safety_init(void);

agent_status_t  agent_policy_safety_validate(const agent_policy_candidate_t *cand);

agent_status_t  agent_policy_safety_clamp_confidence(agent_policy_candidate_t *cand);

bool            agent_policy_safety_allow_transition(agent_state_t from,
                                                     agent_state_t to);

void            agent_policy_safety_escalate_candidate(agent_policy_candidate_t *cand,
                                                       uint32_t reason);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_SAFETY_H */