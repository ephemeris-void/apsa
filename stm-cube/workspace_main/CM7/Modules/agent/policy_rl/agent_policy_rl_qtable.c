#include "agent_policy_rl_qtable.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY RL QTABLE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 1 µs per get/set
 * ============================================================ */

/* PERF: Q-table in DTCM (zero wait state) */
static AGENT_DTCM float g_qtable[AGENT_RL_QTABLE_SIZE];
static bool            g_qtable_initialized = false;

/* ---------- Index helper (bounds-safe) ---------- */
AGENT_INLINE_FAST uint32_t q_index(uint32_t state_idx, uint32_t action_idx)
{
    return (state_idx * AGENT_RL_ACTION_COUNT + action_idx) % AGENT_RL_QTABLE_SIZE;
}

/* ---------- INIT ---------- */
agent_status_t agent_policy_rl_qtable_init(void)
{
    memset(g_qtable, 0, sizeof(g_qtable));
    g_qtable_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_rl_qtable_reset(void)
{
    return agent_policy_rl_qtable_init();
}

/* ---------- GET ---------- */
float agent_policy_rl_qtable_get(uint32_t state_idx, uint32_t action_idx)
{
    if (!g_qtable_initialized)                  return 0.0f;
    if (action_idx >= AGENT_RL_ACTION_COUNT)    return 0.0f;

    return g_qtable[q_index(state_idx, action_idx)];
}

/* ---------- SET ---------- */
agent_status_t agent_policy_rl_qtable_set(uint32_t state_idx,
                                          uint32_t action_idx,
                                          float    value)
{
    if (!g_qtable_initialized)                  return AGENT_ERR_NOT_INIT;
    if (action_idx >= AGENT_RL_ACTION_COUNT)    return AGENT_ERR;

    g_qtable[q_index(state_idx, action_idx)] = value;
    return AGENT_OK;
}

/* ---------- SET DATA (bulk) ---------- */
agent_status_t agent_policy_rl_qtable_set_data(const float *data, uint32_t count)
{
    if (data == NULL)                   return AGENT_ERR;
    if (count > AGENT_RL_QTABLE_SIZE)   return AGENT_ERR;

    for (uint32_t i = 0U; i < count; i++) {
        g_qtable[i] = data[i];
    }
    return AGENT_OK;
}

/* ---------- SIZE ---------- */
uint32_t agent_policy_rl_qtable_size(void)
{
    return AGENT_RL_QTABLE_SIZE;
}