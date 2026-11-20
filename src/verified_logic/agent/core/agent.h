#ifndef AGENT_H
#define AGENT_H

#include "agent_types.h"
#include "agent_config.h"
#include "agent_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT — FROZEN PUBLIC API
 * ============================================================
 * Single entry point. Called from main.c or Zephyr thread.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * ============================================================ */

/* ---------- Lifecycle (FROZEN) ---------- */
agent_status_t  agent_init(void);
agent_status_t  agent_step(void);
agent_status_t  agent_shutdown(void);

/* ---------- State Access (FROZEN) ---------- */
agent_state_t   agent_get_state(void);
const char*     agent_get_state_name(agent_state_t s);
agent_health_t  agent_get_health(void);
float           agent_get_confidence(void);

/* ---------- Diagnostics (FROZEN) ---------- */
uint32_t        agent_get_iteration_count(void);
uint32_t        agent_get_fault_count(void);
uint32_t        agent_get_recovery_count(void);
uint32_t        agent_get_wcet_max_us(void);
uint32_t        agent_get_deadline_miss_count(void);

/* ---------- External Event Injection (FROZEN) ----------
 * These are the ONLY entry points from outside modules.
 * All external modules (IMU, AI, EKF, MPC, IPC) call these.
 * Changing these signatures breaks the entire system.
 */
void            agent_event_imu_fail(void);
void            agent_event_ai_fail(void);
void            agent_event_ekf_diverge(void);
void            agent_event_ipc_timeout(void);
void            agent_event_watchdog(void);
void            agent_event_sensor_recovered(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_H */