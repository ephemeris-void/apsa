#include "agent_policy_safety.h"
#include "agent_policy.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT POLICY SAFETY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per call
 * ============================================================ */

static bool g_policy_safety_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_safety_init(void)
{
    g_policy_safety_initialized = true;
    return AGENT_OK;
}

/* ---------- VALIDATE ---------- */
agent_status_t agent_policy_safety_validate(const agent_policy_candidate_t *cand)
{
    if (cand == NULL) {
        return AGENT_ERR;
    }

    if (!g_policy_safety_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    /* Bounds check */
    if (cand->target_state >= AGENT_STATE_MAX) {
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                        (uint32_t)cand->id,
                        (uint32_t)cand->target_state);
        return AGENT_ERR_INVALID_STATE;
    }

    /* Confidence bounds */
    if (cand->confidence_milli > 1000U) {
        return AGENT_ERR;
    }

    /* VIRTUALIZE/FUSION requires healthy AI */
    if (cand->target_state == AGENT_STATE_VIRTUALIZE ||
        cand->target_state == AGENT_STATE_FUSION) {

        if (ai_health() != AGENT_OK) {
            agent_log_event(AGENT_LOG_AI_FAIL, (uint32_t)cand->id, 0U);
            return AGENT_ERR_HW;
        }
    }

    /* MONITOR/DEGRADED requires functioning IMU */
    if (cand->target_state == AGENT_STATE_MONITOR ||
        cand->target_state == AGENT_STATE_DEGRADED) {

        if (imu_health() == AGENT_HEALTH_FAIL) {
            return AGENT_ERR_HW;
        }
    }

    /* RECOVERY requires healthy IMU + non-diverged EKF */
    if (cand->target_state == AGENT_STATE_RECOVERY) {

        if (imu_health() != AGENT_HEALTH_OK) {
            return AGENT_ERR_HW;
        }

        if (ekf_has_diverged()) {
            return AGENT_ERR;
        }
    }

    return AGENT_OK;
}

/* ---------- CLAMP CONFIDENCE ---------- */
agent_status_t agent_policy_safety_clamp_confidence(agent_policy_candidate_t *cand)
{
    if (cand == NULL) {
        return AGENT_ERR;
    }

    float min_milli = AGENT_CONFIDENCE_MIN * 1000.0f;
    float rec_milli = AGENT_CONFIDENCE_RECOVER * 1000.0f;

    /* FUSION requires high confidence — clamp to min */
    if (cand->target_state == AGENT_STATE_FUSION &&
        (float)cand->confidence_milli < rec_milli) {

        cand->confidence_milli = (uint32_t)min_milli;
    }

    /* Global upper clamp */
    if (cand->confidence_milli > 1000U) {
        cand->confidence_milli = 1000U;
    }

    return AGENT_OK;
}

/* ---------- ALLOW TRANSITION ---------- */
bool agent_policy_safety_allow_transition(agent_state_t from, agent_state_t to)
{
    if (to >= AGENT_STATE_MAX) {
        return false;
    }

    /* EMERGENCY / LANDING always allowed */
    if (to == AGENT_STATE_EMERGENCY || to == AGENT_STATE_LANDING) {
        return true;
    }

    /* EMERGENCY can only go to LANDING */
    if (from == AGENT_STATE_EMERGENCY &&
        to != AGENT_STATE_LANDING) {
        return false;
    }

    /* Envelope must be clean */
    if (!agent_safety_check()) {
        return false;
    }

    return true;
}

/* ---------- ESCALATE CANDIDATE ---------- */
void agent_policy_safety_escalate_candidate(agent_policy_candidate_t *cand,
                                             uint32_t reason)
{
    if (cand == NULL) {
        return;
    }

    cand->priority          = AGENT_PRIO_CRITICAL;
    cand->target_state      = AGENT_STATE_EMERGENCY;
    cand->reason            = (agent_reason_t)reason;
    cand->confidence_milli  = 1000U;
}