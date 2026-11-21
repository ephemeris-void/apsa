#include "agent_goal_monitor.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"
#include <math.h>

/* ============================================================
 * AGENT GOAL MONITOR — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per step
 * ============================================================ */

static bool g_monitor_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_goal_monitor_init(void)
{
    g_monitor_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_goal_monitor_reset(void)
{
    return agent_goal_monitor_init();
}

/* ---------- STEP ---------- */
agent_status_t agent_goal_monitor_step(agent_goal_t *g)
{
    if (g == NULL)              return AGENT_ERR;
    if (!g_monitor_initialized) return AGENT_ERR_NOT_INIT;

    agent_ekf_state_t st;
    if (ekf_get_state(&st) != AGENT_OK) {
        return AGENT_ERR;
    }

    float dx = g->target[0] - st.pos[0];
    float dy = g->target[1] - st.pos[1];
    float dz = g->target[2] - st.pos[2];

    /* PERF: squared compare (no sqrt) */
    float dist2    = dx*dx + dy*dy + dz*dz;
    float tol2     = g->tolerance * g->tolerance;

    if (dist2 < tol2) {
        g->state = AGENT_GOAL_STATE_DONE;
        return AGENT_OK;
    }

    /* Drifted out → back to execution */
    g->state = AGENT_GOAL_STATE_EXECUTING;
    return AGENT_OK;
}