#ifndef AGENT_WATCHDOG_H
#define AGENT_WATCHDOG_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT WATCHDOG — FROZEN
 * ============================================================
 * Self-watchdog: monitors agent loop liveness and WCET.
 * If loop misses kick deadline → escalate.
 * WCET: 10 µs max.
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  agent_watchdog_init(void);

/* ---------- Kick (called each loop iteration) ---------- */
void            agent_watchdog_kick(void);

/* ---------- Check (returns true if OK, false if overdue) ---------- */
bool            agent_watchdog_check(void);

/* ---------- Get Last Kick Timestamp ---------- */
uint32_t        agent_watchdog_last_kick_us(void);

/* ---------- Get Kick Count ---------- */
uint32_t        agent_watchdog_kick_count(void);

/* ---------- Force Reset Counter ---------- */
void            agent_watchdog_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_WATCHDOG_H */