#include "agent_policy_rl_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT POLICY RL SAFETY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per check
 *
 * SAFETY: RL actions MUST pass all gates:
 *   1. Envelope OK
 *   2. Confidence ≥ MIN
 *   3. EKF not diverged
 *   4. Action index in range
 * ============================================================ */

static bool g_rl_safety_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_rl_safety_init(void)
{
    g_rl_safety_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_rl_safety_reset(void)
{
    return agent_policy_rl_safety_init();
}

/* ---------- Envelope check ---------- */
bool agent_policy_rl_safety_check_envelope(void)
{
    return agent_safety_check();
}

/* ---------- Confidence check ---------- */
bool agent_policy_rl_safety_check_confidence(void)
{
    return (agent_confidence_get() >= AGENT_CONFIDENCE_MIN);
}

/* ---------- Master gate ---------- */
bool agent_policy_rl_safety_allow(uint32_t action_idx)
{
    if (!g_rl_safety_initialized)               return false;
    if (action_idx >= AGENT_RL_ACTION_COUNT)    return false;

    if (!agent_policy_rl_safety_check_envelope()) {
        return false;
    }

    if (!agent_policy_rl_safety_check_confidence()) {
        return false;
    }

    if (ekf_has_diverged()) {
        return false;
    }

    return true;
}