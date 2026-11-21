#ifndef AGENT_CONTEXT_H
#define AGENT_CONTEXT_H

#include "agent_types.h"
#include "agent_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT CONTEXT — FROZEN
 * ============================================================
 * Global runtime state. Single instance. No dynamic allocation.
 * Accessed only via agent_context_get().
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * ============================================================ */

/* ---------- Extended Context (full runtime) ---------- */
typedef struct {
    /* State */
    agent_state_t       current_state;
    agent_state_t       previous_state;
    uint32_t            state_entry_us;
    uint32_t            time_in_state_us;

    /* Timing */
    uint32_t            boot_time_us;
    uint32_t            iteration_count;
    uint32_t            loop_start_us;
    uint32_t            loop_end_us;
    uint32_t            loop_wcet_us;
    uint32_t            loop_wcet_max_us;

    /* Health */
    agent_health_t      system_health;
    agent_health_t      imu_health;
    agent_health_t      baro_health;
    agent_health_t      gps_health;
    agent_health_t      ai_health;

    /* Confidence */
    float               confidence;
    float               confidence_min;
    float               confidence_max;

    /* Faults */
    uint32_t            fault_count;
    uint32_t            fault_consecutive;
    uint32_t            fault_last_code;
    uint32_t            fault_last_us;
    uint32_t            recovery_count;
    uint32_t            recovery_attempts;

    /* Priority */
    agent_priority_t    current_priority;

    /* Sensor counters */
    uint32_t            imu_samples_total;
    uint32_t            imu_samples_dropped;
    uint32_t            imu_consecutive_ok;
    uint32_t            imu_consecutive_fail;

    /* AI counters */
    uint32_t            ai_inference_total;
    uint32_t            ai_inference_latency_us;
    uint32_t            ai_inference_latency_max_us;
    uint32_t            ai_consecutive_ok;
    uint32_t            ai_consecutive_fail;

    /* Deadlines */
    uint32_t            deadline_miss_count;
    uint32_t            wcet_violate_count;

    /* Watchdog */
    uint32_t            wdt_kick_count;
    uint32_t            wdt_last_kick_us;

    /* IPC */
    uint32_t            ipc_tx_count;
    uint32_t            ipc_rx_count;
    uint32_t            ipc_last_rx_us;

    /* Recovery */
    uint32_t            recovery_start_us;
    uint32_t            emergency_start_us;

    /* Reserved for future extension */
    uint32_t            reserved[8];
} agent_context_full_t;

/* ---------- Context API (FROZEN) ---------- */
agent_context_full_t* agent_context_get(void);
void                  agent_context_reset(void);
void                  agent_context_init(void);

/* ---------- Iteration Markers ---------- */
void                  agent_iteration_begin(void);
void                  agent_iteration_end(void);

/* ---------- Health Accessors ---------- */
void                  agent_health_set_imu(agent_health_t h);
void                  agent_health_set_baro(agent_health_t h);
void                  agent_health_set_gps(agent_health_t h);
void                  agent_health_set_ai(agent_health_t h);
agent_health_t        agent_health_get_system(void);

/* ---------- Confidence Accessors ---------- */
void                  agent_confidence_set(float c);
float                 agent_confidence_get(void);

/* ---------- Fault Accessors ---------- */
void                  agent_fault_register(agent_fault_t code);
uint32_t              agent_fault_get_count(void);
uint32_t              agent_fault_get_last(void);

/* ---------- Recovery Accessors ---------- */
void                  agent_recovery_register(void);
uint32_t              agent_recovery_get_attempts(void);

/* ---------- Timing Accessors ---------- */
uint32_t              agent_time_now_us(void);
uint32_t              agent_time_in_state_us(void);
uint32_t              agent_time_since_boot_us(void);

/* ---------- Deadline Check ---------- */
bool                  agent_deadline_check(void);

/* ---------- WCET Check ---------- */
bool                  agent_wcet_loop_check(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_CONTEXT_H */