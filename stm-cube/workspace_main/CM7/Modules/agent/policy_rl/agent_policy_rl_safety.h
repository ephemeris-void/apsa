#ifndef AGENT_POLICY_RL_SAFETY_H
#define AGENT_POLICY_RL_SAFETY_H

#include "agent_policy_rl.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY RL SAFETY — FROZEN
 * ============================================================
 * Envelope gate for RL action selection.
 * Blocks RL when envelope violated or confidence low.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per check.
 * ============================================================ */

agent_status_t  agent_policy_rl_safety_init(void);
agent_status_t  agent_policy_rl_safety_reset(void);

bool            agent_policy_rl_safety_allow(uint32_t action_idx);

bool            agent_policy_rl_safety_check_envelope(void);
bool            agent_policy_rl_safety_check_confidence(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_RL_SAFETY_H */