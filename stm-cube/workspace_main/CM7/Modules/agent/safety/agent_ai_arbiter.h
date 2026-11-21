#ifndef AGENT_AI_ARBITER_H
#define AGENT_AI_ARBITER_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT AI ARBITER — FROZEN
 * ============================================================
 * Deterministic arbitration between real IMU and AI virtual data.
 * Blends per reliability, fails over to safe hold if both invalid.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per run.
 * ============================================================ */

#define AGENT_AI_ARB_MAX_SOURCES    4U

/* ---------- Sources ---------- */
typedef enum {
    AGENT_AI_ARB_SRC_NONE       = 0,
    AGENT_AI_ARB_SRC_REAL_IMU   = 1,
    AGENT_AI_ARB_SRC_VIRTUAL_AI = 2,
    AGENT_AI_ARB_SRC_FUSED      = 3,
    AGENT_AI_ARB_SRC_HOLD       = 4
} agent_ai_arb_source_t;

/* ---------- Result ---------- */
typedef enum {
    AGENT_AI_ARB_RESULT_OK          = 0,
    AGENT_AI_ARB_RESULT_DEGRADED    = 1,
    AGENT_AI_ARB_RESULT_FALLBACK    = 2,
    AGENT_AI_ARB_RESULT_FAIL        = 3
} agent_ai_arb_result_t;

/* ---------- Output ---------- */
typedef struct {
    agent_ai_arb_source_t   source;
    agent_ai_arb_result_t   result;
    float                   weight_real;
    float                   weight_ai;
    uint32_t                confidence_milli;
    uint32_t                timestamp_us;
    agent_sensor_sample_t   output;
} agent_ai_arb_out_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_ai_arbiter_init(void);
agent_status_t                  agent_ai_arbiter_reset(void);

agent_status_t                  agent_ai_arbiter_run(const agent_sensor_sample_t *real_sample,
                                                     bool                          real_valid,
                                                     const agent_sensor_sample_t *virtual_sample,
                                                     bool                          virtual_valid,
                                                     agent_ai_arb_out_t           *out);

agent_ai_arb_source_t           agent_ai_arbiter_last_source(void);
const agent_ai_arb_out_t       *agent_ai_arbiter_last(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_AI_ARBITER_H */