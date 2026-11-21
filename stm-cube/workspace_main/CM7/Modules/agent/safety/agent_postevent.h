#ifndef AGENT_POSTEVENT_H
#define AGENT_POSTEVENT_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POST-EVENT — FROZEN
 * ============================================================
 * Records critical events (hardfault, watchdog, emergency)
 * with full state snapshot for post-mortem analysis.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per record.
 * ============================================================ */

#define AGENT_POSTEVENT_MAX_RECORDS     16U
#define AGENT_POSTEVENT_SNAPSHOT_SIZE   32U

/* ---------- Event Types ---------- */
typedef enum {
    AGENT_POSTEVENT_TYPE_NONE       = 0,
    AGENT_POSTEVENT_TYPE_HARDFAULT  = 1,
    AGENT_POSTEVENT_TYPE_WATCHDOG   = 2,
    AGENT_POSTEVENT_TYPE_EMERGENCY  = 3,
    AGENT_POSTEVENT_TYPE_SENSOR     = 4,
    AGENT_POSTEVENT_TYPE_AI_FAIL    = 5,
    AGENT_POSTEVENT_TYPE_WCET       = 6
} agent_postevent_type_t;

/* ---------- Record ---------- */
typedef struct {
    agent_postevent_type_t  type;
    uint32_t                timestamp_us;
    uint32_t                iteration;
    uint32_t                state;
    uint32_t                priority;
    uint32_t                fault_count;
    uint32_t                recovery_count;
    uint32_t                confidence_milli;
    uint32_t                snapshot[AGENT_POSTEVENT_SNAPSHOT_SIZE];
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_postevent_record_t;

/* ---------- State ---------- */
typedef struct {
    agent_postevent_record_t  records[AGENT_POSTEVENT_MAX_RECORDS];
    uint32_t                  head;
    uint32_t                  count;
    uint32_t                  total_events;
    uint32_t                  last_event_us;
    uint32_t                  reserved[2];   /* ABI-safe future extension */
} agent_postevent_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                         agent_postevent_init(void);
agent_status_t                         agent_postevent_reset(void);

agent_status_t                         agent_postevent_record(agent_postevent_type_t type);

const agent_postevent_record_t        *agent_postevent_last(void);
const agent_postevent_record_t        *agent_postevent_get(uint32_t idx);

uint32_t                               agent_postevent_count(void);
void                                   agent_postevent_clear(void);

const agent_postevent_state_t         *agent_postevent_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POSTEVENT_H */