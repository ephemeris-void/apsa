#ifndef AGENT_ESCALATION_H
#define AGENT_ESCALATION_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT ESCALATION — FROZEN
 * ============================================================
 * Multi-level escalation ladder (fault → notice → warning →
 * degraded → critical) with history log.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per update.
 * ============================================================ */

#define AGENT_ESCALATION_MAX_LEVELS     5U
#define AGENT_ESCALATION_HISTORY_DEPTH  16U

/* ---------- Levels ---------- */
typedef enum {
    AGENT_ESCALATION_LEVEL_NONE      = 0,
    AGENT_ESCALATION_LEVEL_NOTICE    = 1,
    AGENT_ESCALATION_LEVEL_WARNING   = 2,
    AGENT_ESCALATION_LEVEL_DEGRADED  = 3,
    AGENT_ESCALATION_LEVEL_CRITICAL  = 4
} agent_escalation_level_t;

/* ---------- Triggers ---------- */
typedef enum {
    AGENT_ESCALATION_TRIGGER_NONE       = 0x00,
    AGENT_ESCALATION_TRIGGER_FAULT      = 0x01,
    AGENT_ESCALATION_TRIGGER_TIMEOUT    = 0x02,
    AGENT_ESCALATION_TRIGGER_DIVERGENCE = 0x03,
    AGENT_ESCALATION_TRIGGER_WCET       = 0x04,
    AGENT_ESCALATION_TRIGGER_WATCHDOG   = 0x05,
    AGENT_ESCALATION_TRIGGER_ENVELOPE   = 0x06,
    AGENT_ESCALATION_TRIGGER_CONFIDENCE = 0x07,
    AGENT_ESCALATION_TRIGGER_MANUAL     = 0x08
} agent_escalation_trigger_t;

/* ---------- Event ---------- */
typedef struct {
    agent_escalation_level_t    level;
    agent_escalation_trigger_t  trigger;
    uint32_t                    reason_code;
    uint32_t                    timestamp_us;
    uint32_t                    fault_count;
} agent_escalation_event_t;

/* ---------- State ---------- */
typedef struct {
    agent_escalation_level_t  current_level;
    agent_escalation_level_t  max_level;
    uint32_t                  escalation_count;
    uint32_t                  deescalation_count;
    uint32_t                  last_change_us;
    uint32_t                  stable_dwell_us;
    uint32_t                  reserved[2];   /* ABI-safe future extension */
} agent_escalation_state_t;

/* ---------- Log ---------- */
typedef struct {
    agent_escalation_event_t  history[AGENT_ESCALATION_HISTORY_DEPTH];
    uint32_t                  head;
    uint32_t                  count;
} agent_escalation_log_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                      agent_escalation_init(void);
agent_status_t                      agent_escalation_reset(void);

agent_status_t                      agent_escalation_raise(agent_escalation_trigger_t trigger,
                                                          uint32_t                   reason_code);

agent_status_t                      agent_escalation_lower(agent_escalation_trigger_t trigger);

agent_escalation_level_t            agent_escalation_current(void);
agent_escalation_level_t            agent_escalation_max(void);

void                                agent_escalation_auto_update(void);

const agent_escalation_state_t     *agent_escalation_state(void);
const agent_escalation_log_t       *agent_escalation_log(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_ESCALATION_H */