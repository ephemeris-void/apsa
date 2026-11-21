#ifndef AGENT_EXPLAIN_H
#define AGENT_EXPLAIN_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT EXPLAIN — FROZEN
 * ============================================================
 * Human-readable explanation log of agent decisions.
 * Records state transitions with reasons for post-flight review.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per record.
 * ============================================================ */

#define AGENT_EXPLAIN_MAX_ENTRIES       16U
#define AGENT_EXPLAIN_TEXT_LEN          48U

/* ---------- Reason categories ---------- */
typedef enum {
    AGENT_EXPLAIN_REASON_NONE       = 0,
    AGENT_EXPLAIN_REASON_STATE      = 1,
    AGENT_EXPLAIN_REASON_HEALTH     = 2,
    AGENT_EXPLAIN_REASON_FAULT      = 3,
    AGENT_EXPLAIN_REASON_TIMEOUT    = 4,
    AGENT_EXPLAIN_REASON_CONFIDENCE = 5,
    AGENT_EXPLAIN_REASON_ENVELOPE   = 6,
    AGENT_EXPLAIN_REASON_MANUAL     = 7
} agent_explain_reason_t;

/* ---------- Entry ---------- */
typedef struct {
    uint32_t                timestamp_us;
    agent_explain_reason_t  reason;
    uint32_t                from_state;
    uint32_t                to_state;
    uint32_t                priority;
    float                   confidence;
    char                    text[AGENT_EXPLAIN_TEXT_LEN];
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_explain_entry_t;

/* ---------- State ---------- */
typedef struct {
    agent_explain_entry_t   entries[AGENT_EXPLAIN_MAX_ENTRIES];
    uint32_t                head;
    uint32_t                count;
    uint32_t                total_entries;
    uint32_t                last_us;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_explain_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                      agent_explain_init(void);
agent_status_t                      agent_explain_reset(void);

agent_status_t                      agent_explain_record(agent_explain_reason_t reason,
                                                        uint32_t                from_state,
                                                        uint32_t                to_state,
                                                        const char             *text);

const agent_explain_entry_t        *agent_explain_last(void);
const agent_explain_entry_t        *agent_explain_get(uint32_t idx);

uint32_t                            agent_explain_count(void);
void                                agent_explain_clear(void);

const agent_explain_state_t        *agent_explain_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_EXPLAIN_H */