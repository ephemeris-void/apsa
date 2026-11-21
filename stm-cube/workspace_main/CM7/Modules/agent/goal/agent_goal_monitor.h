#ifndef AGENT_GOAL_MONITOR_H
#define AGENT_GOAL_MONITOR_H

#include "agent_goal.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT GOAL MONITOR — FROZEN
 * ============================================================
 * Confirms goal completion via monitoring dwell time.
 * Prevents premature completion from single good sample.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per step.
 * ============================================================ */

agent_status_t  agent_goal_monitor_init(void);
agent_status_t  agent_goal_monitor_reset(void);
agent_status_t  agent_goal_monitor_step(agent_goal_t *g);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_GOAL_MONITOR_H */