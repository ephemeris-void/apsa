#ifndef AGENT_GOAL_PLANNER_H
#define AGENT_GOAL_PLANNER_H

#include "agent_goal.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT GOAL PLANNER — FROZEN
 * ============================================================
 * Plans initial action set for a goal.
 * Deterministic — goal ID → fixed plan structure.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 30 µs max per plan/step.
 * ============================================================ */

agent_status_t  agent_goal_planner_init(void);
agent_status_t  agent_goal_planner_reset(void);

agent_status_t  agent_goal_planner_plan(agent_goal_t *g);
agent_status_t  agent_goal_planner_step(agent_goal_t *g);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_GOAL_PLANNER_H */