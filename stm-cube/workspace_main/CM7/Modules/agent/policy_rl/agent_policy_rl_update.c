#include "agent_policy_rl_update.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT POLICY RL UPDATE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 2 µs per call
 * ============================================================ */

/* ---------- Q-learning update ----------
 * Q_new = Q_old + α * (reward + γ * max_Q_next − Q_old)
 */
float agent_policy_rl_update_q(float q_old,
                                                 float reward,
                                                 float max_q_next,
                                                 float alpha,
                                                 float gamma)
{
    return q_old + alpha * (reward + gamma * max_q_next - q_old);
}

/* ---------- Reward: from system health ---------- */
float agent_policy_rl_update_reward_from_health(agent_health_t h)
{
    switch (h) {
    case AGENT_HEALTH_OK:       return  1.0f;
    case AGENT_HEALTH_DEGRADED: return -0.5f;
    case AGENT_HEALTH_FAIL:     return -1.0f;
    default:                    return  0.0f;
    }
}

/* ---------- Reward: from confidence ---------- */
float agent_policy_rl_update_reward_from_confidence(float c)
{
    if (c >= AGENT_CONFIDENCE_RECOVER) return  1.0f;
    if (c >= AGENT_CONFIDENCE_MIN)     return  0.0f;
    return -1.0f;
}

/* ---------- Reward: from envelope ---------- */
float agent_policy_rl_update_reward_from_envelope(bool in_envelope)
{
    return in_envelope ? 1.0f : -1.0f;
}