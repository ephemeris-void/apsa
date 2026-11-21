#ifndef AGENT_POLICY_RL_H
#define AGENT_POLICY_RL_H

#include "../core/agent_types.h"
#include "../policy/agent_policy_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY RL — FROZEN
 * ============================================================
 * Tabular Q-learning policy over discretized state space.
 * SAFETY: RL output is envelope-checked before application.
 *         Online updates are frozen by default.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 100 µs max per select/update.
 * ============================================================ */

#define AGENT_RL_STATE_BINS         16U
#define AGENT_RL_STATE_DIM          4U
#define AGENT_RL_ACTION_COUNT       AGENT_POLICY_ID_COUNT
#define AGENT_RL_QTABLE_SIZE        (AGENT_RL_STATE_BINS * AGENT_RL_STATE_DIM * AGENT_RL_ACTION_COUNT)

#define AGENT_RL_ALPHA              0.10f
#define AGENT_RL_GAMMA              0.95f
#define AGENT_RL_EPSILON            0.05f
#define AGENT_RL_MIN_CONFIDENCE     0.70f

/* ---------- Step record ---------- */
typedef struct {
    float    state[AGENT_RL_STATE_DIM];
    uint32_t state_idx;
    uint32_t action_idx;
    float    reward;
    float    q_value;
    float    max_q_next;
    uint32_t step_count;
    uint32_t reserved[2];   /* ABI-safe future extension */
} agent_policy_rl_step_t;

/* ---------- Context ---------- */
typedef struct {
    float                    qtable[AGENT_RL_QTABLE_SIZE];
    agent_policy_rl_step_t   last_step;
    uint32_t                 total_steps;
    uint32_t                 explore_count;
    uint32_t                 exploit_count;
    uint32_t                 safety_block_count;
    uint32_t                 last_update_us;
    bool                     enabled;
    bool                     frozen;
    uint32_t                 reserved[2];   /* ABI-safe future extension */
} agent_policy_rl_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_policy_rl_init(void);
agent_status_t              agent_policy_rl_reset(void);
agent_status_t              agent_policy_rl_enable(bool en);

agent_status_t              agent_policy_rl_discretize(const float state[AGENT_RL_STATE_DIM],
                                                       uint32_t   *state_idx_out);

agent_status_t              agent_policy_rl_select(uint32_t *action_out);

agent_status_t              agent_policy_rl_update(uint32_t next_state_idx,
                                                   float    reward);

agent_status_t              agent_policy_rl_load_qtable(const float *qtable,
                                                        uint32_t     count);

float                       agent_policy_rl_q(uint32_t state_idx,
                                               uint32_t action_idx);

const agent_policy_rl_ctx_t *agent_policy_rl_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_RL_H */