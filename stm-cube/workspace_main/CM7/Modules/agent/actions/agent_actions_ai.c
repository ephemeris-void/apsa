#include "agent_actions_ai.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT AI-SPECIFIC ACTIONS — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs per action
 *
 * SAFETY: Non-blocking AI trigger. ai_trigger() sets RUNNING
 *         flag; ai_model_poll() runs inference from agent loop.
 * ============================================================ */

/* ---------- TRIGGER INFERENCE ---------- */
agent_status_t agent_actions_ai_trigger(void)
{
    if (ai_status() == AI_STATUS_RUNNING) {
        return AGENT_OK;    /* already queued */
    }

    if (ai_status() == AI_STATUS_READY) {
        return AGENT_OK;    /* output waiting to be consumed */
    }

    agent_status_t st = ai_trigger();

    if (st == AGENT_OK) {
        agent_log_event(AGENT_LOG_AI_TRIGGER, 0U, 0U);
    } else {
        agent_health_set_ai(AGENT_HEALTH_FAIL);
        agent_log_event(AGENT_LOG_AI_FAIL, 0U, 0U);
    }

    return st;
}

/* ---------- ABORT INFERENCE ---------- */
agent_status_t agent_actions_ai_abort(void)
{
    /* X-CUBE-AI runs to completion; no abort supported.
     * Reserved for future interrupt-based AI. */
    agent_log_event(AGENT_LOG_AI_ABORT, 0U, 0U);
    return AGENT_OK;
}

/* ---------- FUSE AI OUTPUT WITH EKF ---------- */
agent_status_t agent_actions_ai_fuse(void)
{
    if (!ai_output_ready()) {
        return AGENT_ERR;
    }

    agent_sensor_sample_t sample;
    agent_status_t st = ai_get_output(&sample);

    if (st != AGENT_OK) {
        return st;
    }

    sample.health       = AGENT_HEALTH_OK;
    sample.timestamp_us = agent_time_now_us();

    return ekf_update(&sample);
}

/* ---------- SWITCH AI INPUT SOURCE ---------- */
agent_status_t agent_actions_ai_switch_input(uint32_t source_id)
{
    /* Reserved for multi-source AI input (IMU, Baro, GPS).
     * Current AI model uses IMU only. */
    agent_log_event(AGENT_LOG_AI_SWITCH_INPUT, source_id, 0U);
    return AGENT_OK;
}

/* ---------- READ AI OUTPUT SAMPLE ---------- */
agent_status_t agent_actions_ai_read_output(agent_sensor_sample_t *out)
{
    if (out == NULL) {
        return AGENT_ERR;
    }

    if (!ai_output_ready()) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_status_t st = ai_get_output(out);
    if (st == AGENT_OK) {
        out->health       = AGENT_HEALTH_OK;
        out->timestamp_us = agent_time_now_us();
    }

    return st;
}

/* ---------- CHECK CONFIDENCE ---------- */
bool agent_actions_ai_is_confident(void)
{
    return (agent_confidence_get() >= AGENT_AI_CONFIDENCE_THRESHOLD);
}