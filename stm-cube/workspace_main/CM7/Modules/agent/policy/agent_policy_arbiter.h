#ifndef AGENT_POLICY_ARBITER_H
#define AGENT_POLICY_ARBITER_H

#include "agent_policy_if.h"
#include "agent_policy.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY ARBITER — FROZEN
 * ============================================================
 * Collect → filter → arbitrate → apply.
 * Deterministic. No randomness. Bounded WCET.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 50 µs max per run.
 * ============================================================ */

agent_status_t                     agent_policy_arbiter_reset(void);
agent_status_t                     agent_policy_arbiter_run(agent_policy_candidate_t *winner_out);
const agent_policy_arbitration_t  *agent_policy_arbiter_last(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_ARBITER_H */