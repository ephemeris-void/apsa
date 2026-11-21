#ifndef AGENT_AI_SAFETY_H
#define AGENT_AI_SAFETY_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT AI SAFETY — FROZEN
 * ============================================================
 * Envelope + freshness + confidence gate for AI output.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per validate.
 * ============================================================ */

/* ---------- Result ---------- */
typedef enum {
    AGENT_AI_SAFETY_OK            = 0,
    AGENT_AI_SAFETY_DEGRADED      = 1,
    AGENT_AI_SAFETY_REJECTED      = 2,
    AGENT_AI_SAFETY_STALE         = 3,
    AGENT_AI_SAFETY_ENVELOPE_FAIL = 4,
    AGENT_AI_SAFETY_DIVERGED      = 5
} agent_ai_safety_result_t;

/* ---------- Report ---------- */
typedef struct {
    agent_ai_safety_result_t  result;
    uint32_t                  reason_code;
    uint32_t                  timestamp_us;
    uint32_t                  wcet_us;
    float                     confidence;
    float                     envelope_margin;
    bool                      accepted;
    uint32_t                  reserved[2];   /* future extension */
} agent_ai_safety_report_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_ai_safety_init(void);
agent_status_t              agent_ai_safety_reset(void);

agent_status_t              agent_ai_safety_validate(const agent_sensor_sample_t *sample,
                                                     agent_ai_safety_report_t   *report_out);

agent_status_t              agent_ai_safety_check_envelope(const agent_sensor_sample_t *sample,
                                                           float                       *margin_out);

agent_status_t              agent_ai_safety_check_freshness(uint32_t last_update_us,
                                                            uint32_t max_age_us);

agent_status_t              agent_ai_safety_check_confidence(float    confidence,
                                                             float    min_confidence);

bool                        agent_ai_safety_is_healthy(void);

const agent_ai_safety_report_t *agent_ai_safety_last(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_AI_SAFETY_H */