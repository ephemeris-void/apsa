#ifndef AGENT_ACTIONS_AI_H
#define AGENT_ACTIONS_AI_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT AI-SPECIFIC ACTIONS — FROZEN
 * ============================================================
 * AI-related actions separated from general actions.
 * WCET: 50 µs max per action.
 * ============================================================ */

/* ---------- Trigger Inference ---------- */
agent_status_t  agent_actions_ai_trigger(void);

/* ---------- Abort Inference ---------- */
agent_status_t  agent_actions_ai_abort(void);

/* ---------- Fuse AI Output with EKF ---------- */
agent_status_t  agent_actions_ai_fuse(void);

/* ---------- Switch AI Input Source ---------- */
agent_status_t  agent_actions_ai_switch_input(uint32_t source_id);

/* ---------- Read AI Output Sample ---------- */
agent_status_t  agent_actions_ai_read_output(agent_sensor_sample_t *out);

/* ---------- Check AI Confidence ---------- */
bool            agent_actions_ai_is_confident(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_ACTIONS_AI_H */