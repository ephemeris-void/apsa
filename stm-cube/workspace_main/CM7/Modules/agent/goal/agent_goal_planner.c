#include "agent_goal_planner.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT GOAL PLANNER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 30 µs per plan/step
 * ============================================================ */

static bool g_planner_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_goal_planner_init(void)
{
    g_planner_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_goal_planner_reset(void)
{
    return agent_goal_planner_init();
}

/* ---------- PLAN ----------
 * Deterministic per-goal-ID planning.
 * Sets initial state directly to EXECUTING.
 */
agent_status_t agent_goal_planner_plan(agent_goal_t *g)
{
    if (g == NULL)               return AGENT_ERR;
    if (!g_planner_initialized)  return AGENT_ERR_NOT_INIT;

    g->state      = AGENT_GOAL_STATE_EXECUTING;
    g->step_count = 0U;

    return AGENT_OK;
}

/* ---------- STEP ----------
 * Planning iteration: increments step, escalates if too many.
 */
agent_status_t agent_goal_planner_step(agent_goal_t *g)
{
    if (g == NULL) return AGENT_ERR;

    g->step_count++;

    if (g->step_count >= AGENT_GOAL_MAX_STEPS) {
        g->state = AGENT_GOAL_STATE_FAILED;
        return AGENT_ERR;
    }

    g->state = AGENT_GOAL_STATE_EXECUTING;
    return AGENT_OK;
}