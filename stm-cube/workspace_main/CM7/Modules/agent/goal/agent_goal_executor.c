#include "agent_goal_executor.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"
#include <math.h>

/* ============================================================
 * AGENT GOAL EXECUTOR — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 30 µs per step
 * ============================================================ */

static bool g_executor_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_goal_executor_init(void)
{
    g_executor_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_goal_executor_reset(void)
{
    return agent_goal_executor_init();
}

/* ---------- STEP ---------- */
agent_status_t agent_goal_executor_step(agent_goal_t *g)
{
    if (g == NULL)               return AGENT_ERR;
    if (!g_executor_initialized) return AGENT_ERR_NOT_INIT;

    agent_ekf_state_t st;
    if (ekf_get_state(&st) != AGENT_OK) {
        return AGENT_ERR;
    }

    float dx = g->target[0] - st.pos[0];
    float dy = g->target[1] - st.pos[1];
    float dz = g->target[2] - st.pos[2];

    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    /* Progress calculation (milli-scale) */
    if (dist < 1e-3f) {
        g->progress_milli = 1000U;
    } else {
        float initial = sqrtf(g->target[0]*g->target[0] +
                              g->target[1]*g->target[1] +
                              g->target[2]*g->target[2]);

        if (initial < 1e-3f) {
            initial = 1e-3f;
        }

        float progress = 1.0f - (dist / initial);

        if (progress < 0.0f) progress = 0.0f;
        if (progress > 1.0f) progress = 1.0f;

        g->progress_milli = (uint32_t)(progress * 1000.0f);
    }

    /* Completion: within tolerance */
    if (dist < g->tolerance) {
        g->state = AGENT_GOAL_STATE_MONITORING;
    }

    /* Deadline enforcement */
    uint32_t now = agent_time_now_us();

    if (g->deadline_us > 0U &&
        (now - g->start_us) > g->deadline_us) {
        g->state = AGENT_GOAL_STATE_FAILED;
        return AGENT_ERR_TIMEOUT;
    }

    return AGENT_OK;
}