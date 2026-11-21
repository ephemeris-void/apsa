#include "agent_policy_rl.h"
#include "agent_policy_rl_qtable.h"
#include "agent_policy_rl_update.h"
#include "agent_policy_rl_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY RL — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 100 µs per select/update
 *
 * SAFETY: RL frozen by default. Online updates disabled until
 *         explicitly enabled — prevents reward hacking.
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_rl_ctx_t g_rl;
static bool                             g_rl_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_rl_init(void)
{
    memset(&g_rl, 0, sizeof(g_rl));    /* zero-init incl. reserved[] */

    agent_policy_rl_qtable_init();
    agent_policy_rl_safety_init();

    g_rl.total_steps        = 0U;
    g_rl.explore_count      = 0U;
    g_rl.exploit_count      = 0U;
    g_rl.safety_block_count = 0U;
    g_rl.last_update_us     = agent_time_now_us();
    g_rl.enabled            = false;
    g_rl.frozen             = true;

    g_rl_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_rl_reset(void)
{
    g_rl_initialized = false;
    return agent_policy_rl_init();
}

/* ---------- ENABLE ---------- */
agent_status_t agent_policy_rl_enable(bool en)
{
    g_rl.enabled = en;
    g_rl.frozen  = !en;
    return AGENT_OK;
}

/* ---------- DISCRETIZE ---------- */
agent_status_t agent_policy_rl_discretize(const float state[AGENT_RL_STATE_DIM],
                                          uint32_t   *state_idx_out)
{
    if (state == NULL || state_idx_out == NULL) {
        return AGENT_ERR;
    }

    uint32_t idx  = 0U;
    uint32_t mult = 1U;

    for (uint32_t d = 0U; d < AGENT_RL_STATE_DIM; d++) {

        float v = state[d];

        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;

        uint32_t bin = (uint32_t)(v * (float)(AGENT_RL_STATE_BINS - 1U));

        if (bin >= AGENT_RL_STATE_BINS) {
            bin = AGENT_RL_STATE_BINS - 1U;
        }

        idx  += bin * mult;
        mult *= AGENT_RL_STATE_BINS;
    }

    /* Modulo to fit table range (bounded) */
    uint32_t state_range = AGENT_RL_QTABLE_SIZE / AGENT_RL_ACTION_COUNT;
    idx = idx % state_range;

    *state_idx_out = idx;
    return AGENT_OK;
}

/* ---------- SELECT (greedy argmax + safety gate) ---------- */
agent_status_t agent_policy_rl_select(uint32_t *action_out)
{
    if (action_out == NULL)     return AGENT_ERR;
    if (!g_rl_initialized)      return AGENT_ERR_NOT_INIT;

    if (!g_rl.enabled || g_rl.frozen) {
        return AGENT_ERR_INVALID_STATE;
    }

    uint32_t s_idx = g_rl.last_step.state_idx;

    /* Greedy argmax */
    uint32_t best_action = 0U;
    float    best_q      = agent_policy_rl_qtable_get(s_idx, 0U);

    for (uint32_t a = 1U; a < AGENT_RL_ACTION_COUNT; a++) {
        float q = agent_policy_rl_qtable_get(s_idx, a);
        if (q > best_q) {
            best_q      = q;
            best_action = a;
        }
    }

    /* SAFETY: envelope check before allowing RL output */
    if (!agent_policy_rl_safety_allow(best_action)) {
        g_rl.safety_block_count++;
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0xC101U, best_action);
        return AGENT_ERR;
    }

    g_rl.last_step.action_idx = best_action;
    g_rl.last_step.q_value    = best_q;
    *action_out = best_action;

    g_rl.exploit_count++;
    return AGENT_OK;
}

/* ---------- UPDATE (frozen until enabled) ---------- */
agent_status_t agent_policy_rl_update(uint32_t next_state_idx, float reward)
{
    if (!g_rl_initialized)  return AGENT_ERR_NOT_INIT;
    if (g_rl.frozen)        return AGENT_ERR_INVALID_STATE;

    uint32_t s_idx = g_rl.last_step.state_idx;
    uint32_t a_idx = g_rl.last_step.action_idx;

    /* Max Q at next state */
    float max_q_next = 0.0f;

    for (uint32_t a = 0U; a < AGENT_RL_ACTION_COUNT; a++) {
        float q = agent_policy_rl_qtable_get(next_state_idx, a);
        if (q > max_q_next) {
            max_q_next = q;
        }
    }

    float q_old = agent_policy_rl_qtable_get(s_idx, a_idx);

    float q_new = agent_policy_rl_update_q(q_old,
                                           reward,
                                           max_q_next,
                                           AGENT_RL_ALPHA,
                                           AGENT_RL_GAMMA);

    (void)agent_policy_rl_qtable_set(s_idx, a_idx, q_new);

    g_rl.last_step.reward     = reward;
    g_rl.last_step.max_q_next = max_q_next;
    g_rl.last_step.q_value    = q_new;
    g_rl.last_step.step_count++;

    g_rl.total_steps++;
    g_rl.last_update_us = agent_time_now_us();

    return AGENT_OK;
}

/* ---------- LOAD Q-TABLE ---------- */
agent_status_t agent_policy_rl_load_qtable(const float *qtable, uint32_t count)
{
    if (qtable == NULL)                             return AGENT_ERR;
    if (count > AGENT_RL_QTABLE_SIZE)               return AGENT_ERR;

    for (uint32_t i = 0U; i < count; i++) {
        g_rl.qtable[i] = qtable[i];
    }

    return agent_policy_rl_qtable_set_data(g_rl.qtable, count);
}

/* ---------- Q QUERY ---------- */
float agent_policy_rl_q(uint32_t state_idx, uint32_t action_idx)
{
    return agent_policy_rl_qtable_get(state_idx, action_idx);
}

/* ---------- CONTEXT ---------- */
const agent_policy_rl_ctx_t *agent_policy_rl_ctx(void)
{
    return &g_rl;
}