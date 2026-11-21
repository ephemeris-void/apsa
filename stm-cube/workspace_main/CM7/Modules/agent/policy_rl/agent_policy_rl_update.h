#ifndef AGENT_POLICY_RL_UPDATE_H
#define AGENT_POLICY_RL_UPDATE_H

#include "agent_policy_rl.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY RL UPDATE — FROZEN
 * ============================================================
 * Q-learning update + reward shaping helpers.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 2 µs max per call.
 * ============================================================ */

float   agent_policy_rl_update_q(float q_old,
                                 float reward,
                                 float max_q_next,
                                 float alpha,
                                 float gamma);

float   agent_policy_rl_update_reward_from_health(agent_health_t h);
float   agent_policy_rl_update_reward_from_confidence(float c);
float   agent_policy_rl_update_reward_from_envelope(bool in_envelope);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_RL_UPDATE_H */