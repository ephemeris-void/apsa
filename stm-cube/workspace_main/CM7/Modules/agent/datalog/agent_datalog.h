#ifndef AGENT_DATALOG_H
#define AGENT_DATALOG_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT DATALOG — FROZEN
 * ============================================================
 * Lock-free ring buffer for training data capture.
 * Preserves sensor/AI/EKF traces for post-flight ML retraining.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per write.
 * ============================================================ */

#define AGENT_DATALOG_RING_SIZE         512U
#define AGENT_DATALOG_ENTRY_MAX_PAYLOAD 24U

/* ---------- Entry types ---------- */
typedef enum {
    AGENT_DATALOG_NONE          = 0x0000,
    AGENT_DATALOG_SENSOR        = 0x0001,
    AGENT_DATALOG_AI_INPUT      = 0x0002,
    AGENT_DATALOG_AI_OUTPUT     = 0x0003,
    AGENT_DATALOG_EKF_STATE     = 0x0004,
    AGENT_DATALOG_CONTROL_OUT   = 0x0005,
    AGENT_DATALOG_STATE_CHANGE  = 0x0006,
    AGENT_DATALOG_FAULT         = 0x0007,
    AGENT_DATALOG_EVENT         = 0x0008
} agent_datalog_type_t;

/* ---------- Entry ---------- */
typedef struct {
    uint32_t              type;
    uint32_t              timestamp_us;
    uint32_t              arg1;
    uint32_t              arg2;
    uint8_t               payload[AGENT_DATALOG_ENTRY_MAX_PAYLOAD];
    uint8_t               payload_size;
    uint8_t               reserved[3];
} agent_datalog_entry_t;

/* ---------- State ---------- */
typedef struct {
    agent_datalog_entry_t  ring[AGENT_DATALOG_RING_SIZE];
    volatile uint32_t      head;
    volatile uint32_t      tail;
    volatile uint32_t      count;
    uint32_t               total_written;
    uint32_t               total_dropped;
    uint32_t               last_write_us;
    bool                   enabled;
    uint32_t               reserved[2];   /* ABI-safe future extension */
} agent_datalog_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_datalog_init(void);
agent_status_t                  agent_datalog_reset(void);
agent_status_t                  agent_datalog_enable(bool en);

agent_status_t                  agent_datalog_write(agent_datalog_type_t type,
                                                    uint32_t             arg1,
                                                    uint32_t             arg2);

agent_status_t                  agent_datalog_write_payload(agent_datalog_type_t type,
                                                            const uint8_t       *payload,
                                                            uint8_t              payload_size);

agent_status_t                  agent_datalog_read(agent_datalog_entry_t *out);
uint32_t                        agent_datalog_count(void);
void                            agent_datalog_clear(void);

const agent_datalog_state_t    *agent_datalog_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_DATALOG_H */