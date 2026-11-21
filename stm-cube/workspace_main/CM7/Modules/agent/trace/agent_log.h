#ifndef AGENT_LOG_H
#define AGENT_LOG_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT LOG — FROZEN
 * ============================================================
 * Lock-free ring-buffer event log.
 * Preserves events across resets (backup region).
 * Board: NUCLEO-H755ZI-Q
 * WCET: 5 µs max per call.
 * ============================================================ */

/* ---------- Event Codes ---------- */
typedef enum {
    AGENT_LOG_NONE                = 0x0000,

    /* Lifecycle */
    AGENT_LOG_INIT                = 0x0001,
    AGENT_LOG_SHUTDOWN            = 0x0002,
    AGENT_LOG_TRACE_INIT          = 0x0003,
    AGENT_LOG_TRACE_DUMP          = 0x0004,
    AGENT_LOG_WDT_INIT            = 0x0005,
    AGENT_LOG_SAFETY_INIT         = 0x0006,
    AGENT_LOG_SELFTEST_OK         = 0x0007,
    AGENT_LOG_SELFTEST_FAIL       = 0x0008,

    /* State */
    AGENT_LOG_STATE_ENTER         = 0x0010,
    AGENT_LOG_STATE_EXIT          = 0x0011,
    AGENT_LOG_TRANSITION          = 0x0012,
    AGENT_LOG_TRANSITION_FORCED   = 0x0013,
    AGENT_LOG_TRANSITION_BLOCKED  = 0x0014,

    /* Sensors */
    AGENT_LOG_IMU_FAIL            = 0x0020,
    AGENT_LOG_AI_FAIL             = 0x0021,
    AGENT_LOG_EKF_DIVERGE         = 0x0022,
    AGENT_LOG_SENSOR_RECOVERED    = 0x0023,
    AGENT_LOG_ENVELOPE_VIOLATE    = 0x0024,

    /* AI */
    AGENT_LOG_AI_TRIGGER          = 0x0030,
    AGENT_LOG_AI_ABORT            = 0x0031,
    AGENT_LOG_AI_SWITCH_INPUT     = 0x0032,

    /* Action */
    AGENT_LOG_EKF_RESET           = 0x0040,
    AGENT_LOG_EMERGENCY           = 0x0041,
    AGENT_LOG_PERSIST             = 0x0042,

    /* Faults */
    AGENT_LOG_DEADLINE_MISS       = 0x0050,
    AGENT_LOG_WCET_VIOLATE        = 0x0051,
    AGENT_LOG_WDT_TIMEOUT         = 0x0052,
    AGENT_LOG_IPC_TIMEOUT         = 0x0053,
    AGENT_LOG_SAFETY_ESCALATE     = 0x0054,
    AGENT_LOG_WATCHDOG            = 0x0055,

    /* Ground */
    AGENT_LOG_GROUND_ALERT        = 0x0060,
    AGENT_LOG_IPC_SEND            = 0x0061,
    AGENT_LOG_IPC_RECV            = 0x0062
} agent_log_code_t;

/* ---------- Log Entry ---------- */
typedef struct {
    uint32_t timestamp_us;
    uint16_t code;
    uint16_t reserved;
    uint32_t arg1;
    uint32_t arg2;
} agent_log_entry_t;

/* ---------- Log API (FROZEN) ---------- */
agent_status_t  agent_log_init(void);
void            agent_log_event(uint32_t code, uint32_t arg1, uint32_t arg2);
uint32_t        agent_log_count(void);
const agent_log_entry_t* agent_log_get(uint32_t index);
void            agent_log_dump_all(void);
void            agent_log_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_LOG_H */