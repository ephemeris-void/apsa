#ifndef AGENT_AI_IF_H
#define AGENT_AI_IF_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AI INTERFACE — FROZEN
 * ============================================================
 * External module: X-CUBE-AI Model (Modules/ai/)
 * Link point:      ai_icm_virtual.c / ai_icm_virtual.h
 * Model:           int8 virtual sensor autoencoder
 * Board:           NUCLEO-H755ZI-Q
 *
 * To replace AI model (e.g., new architecture, new data):
 *   1. Regenerate via CubeMX
 *   2. This interface stays UNCHANGED
 *   3. Agent code stays UNCHANGED
 *
 * Model input/output contract:
 *   Input:  [1, 4, 512, 1] int8, 2048 bytes
 *   Output: [1, 4, 512, 1] int8, 2048 bytes
 *
 * WCET budget:
 *   ai_init         : 10 ms (boot only)
 *   ai_trigger      : 11 ms (blocking, background)
 *   ai_status       : 1 µs
 *   ai_output_ready : 1 µs
 *   ai_get_output   : 20 µs
 *   ai_health       : 1 µs
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  ai_init(void);

/* ---------- Trigger Inference ----------
 * Starts AI inference. Returns when inference completed.
 * Poll ai_output_ready() for readiness. */
agent_status_t  ai_trigger(void);

/* ---------- Status Query ---------- */
ai_status_t     ai_status(void);

/* ---------- Output Ready Check ---------- */
bool            ai_output_ready(void);

/* ---------- Get Output Sample ----------
 * Extracts first-sample data from AI output buffer.
 * Converts int8 → sensor sample. */
agent_status_t  ai_get_output(agent_sensor_sample_t *sample);

/* ---------- Health ---------- */
agent_status_t  ai_health(void);

/* ---------- Failure Injection ---------- */
void            ai_on_failure(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_AI_IF_H */