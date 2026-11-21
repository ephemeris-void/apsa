#ifndef AGENT_GOAL_EXECUTOR_H
#define AGENT_GOAL_EXECUTOR_H

#include "agent_goal.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT GOAL EXECUTOR — FROZEN
 * ============================================================
 * Executes a goal: computes progress, detects completion/failure.
 * Uses EKF state as ground truth.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 30 µs max per step.
 * ============================================================ */

agent_status_t  agent_goal_executor_init(void);
agent_status_t  agent_goal_executor_reset(void);
agent_status_t  agent_goal_executor_step(agent_goal_t *g);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_GOAL_EXECUTOR_H */