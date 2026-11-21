#ifndef AGENT_POLICY_RL_QTABLE_H
#define AGENT_POLICY_RL_QTABLE_H

#include "agent_policy_rl.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY RL QTABLE — FROZEN
 * ============================================================
 * Dense Q-table storage with bounds-checked access.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 1 µs max per get/set.
 * ============================================================ */

agent_status_t  agent_policy_rl_qtable_init(void);
agent_status_t  agent_policy_rl_qtable_reset(void);

float           agent_policy_rl_qtable_get(uint32_t state_idx, uint32_t action_idx);
agent_status_t  agent_policy_rl_qtable_set(uint32_t state_idx, uint32_t action_idx, float value);

agent_status_t  agent_policy_rl_qtable_set_data(const float *data, uint32_t count);

uint32_t        agent_policy_rl_qtable_size(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_RL_QTABLE_H */