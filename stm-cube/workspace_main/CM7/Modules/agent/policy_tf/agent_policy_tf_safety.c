#include "agent_policy_tf_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT POLICY TF SAFETY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per check
 *
 * SAFETY: TF actions MUST pass all gates:
 *   1. Envelope OK
 *   2. Confidence ≥ MIN
 *   3. EKF not diverged
 *   4. Action index in range
 * ============================================================ */

static bool g_tf_safety_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_tf_safety_init(void)
{
    g_tf_safety_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_tf_safety_reset(void)
{
    return agent_policy_tf_safety_init();
}

/* ---------- MASTER GATE ---------- */
bool agent_policy_tf_safety_check(void)
{
    if (!g_tf_safety_initialized) {
        return false;
    }

    if (!agent_safety_check()) {
        return false;
    }

    if (agent_confidence_get() < AGENT_CONFIDENCE_MIN) {
        return false;
    }

    if (ekf_has_diverged()) {
        return false;
    }

    return true;
}

/* ---------- OUTPUT VALIDATION ---------- */
bool agent_policy_tf_safety_allow_output(uint32_t action_idx)
{
    if (action_idx >= AGENT_TF_OUTPUT_DIM) {
        return false;
    }
    return agent_policy_tf_safety_check();
}