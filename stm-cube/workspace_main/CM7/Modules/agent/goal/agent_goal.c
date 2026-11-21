#include "agent_goal.h"
#include "agent_goal_planner.h"
#include "agent_goal_executor.h"
#include "agent_goal_monitor.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT GOAL — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 100 µs per update
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_goal_state_ctx_t g_goal;
static bool                              g_goal_initialized = false;

/* ---------- Find goal by ID ---------- */
static int32_t find_goal_index(agent_goal_id_t id)
{
    for (uint32_t i = 0U; i < AGENT_GOAL_MAX_ACTIVE; i++) {
        if (g_goal.active[i].active && g_goal.active[i].id == id) {
            return (int32_t)i;
        }
    }
    return -1;
}

/* ---------- Find free slot ---------- */
static int32_t find_free_slot(void)
{
    for (uint32_t i = 0U; i < AGENT_GOAL_MAX_ACTIVE; i++) {
        if (!g_goal.active[i].active) {
            return (int32_t)i;
        }
    }
    return -1;
}

/* ---------- INIT ---------- */
agent_status_t agent_goal_init(void)
{
    memset(&g_goal, 0, sizeof(g_goal));    /* zero-init incl. reserved[] */

    for (uint32_t i = 0U; i < AGENT_GOAL_MAX_ACTIVE; i++) {
        g_goal.active[i].id       = AGENT_GOAL_ID_NONE;
        g_goal.active[i].state    = AGENT_GOAL_STATE_IDLE;
        g_goal.active[i].priority = AGENT_GOAL_PRIO_LOW;
        g_goal.active[i].active   = false;
    }

    g_goal.active_count   = 0U;
    g_goal.current        = AGENT_GOAL_ID_NONE;
    g_goal.previous       = AGENT_GOAL_ID_NONE;
    g_goal.completed      = 0U;
    g_goal.failed         = 0U;
    g_goal.aborted        = 0U;
    g_goal.last_update_us = agent_time_now_us();

    agent_goal_planner_init();
    agent_goal_executor_init();
    agent_goal_monitor_init();

    g_goal_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_goal_reset(void)
{
    g_goal_initialized = false;
    return agent_goal_init();
}

/* ---------- SET ---------- */
agent_status_t agent_goal_set(agent_goal_id_t   id,
                              agent_goal_prio_t prio,
                              const float       target[3],
                              float             tolerance,
                              uint32_t          deadline_us)
{
    if (!g_goal_initialized)                                     return AGENT_ERR_NOT_INIT;
    if (id == AGENT_GOAL_ID_NONE || id >= AGENT_GOAL_ID_COUNT)   return AGENT_ERR;

    int32_t idx = find_goal_index(id);

    if (idx < 0) {
        idx = find_free_slot();
        if (idx < 0) {
            return AGENT_ERR;
        }
        g_goal.active_count++;
    }

    agent_goal_t *g = &g_goal.active[idx];

    g->id             = id;
    g->state          = AGENT_GOAL_STATE_PLANNING;
    g->priority       = prio;
    g->step_count     = 0U;
    g->start_us       = agent_time_now_us();
    g->deadline_us    = deadline_us;
    g->tolerance      = tolerance;
    g->progress_milli = 0U;
    g->active         = true;

    if (target != NULL) {
        g->target[0] = target[0];
        g->target[1] = target[1];
        g->target[2] = target[2];
    } else {
        g->target[0] = 0.0f;
        g->target[1] = 0.0f;
        g->target[2] = 0.0f;
    }

    g_goal.previous = g_goal.current;
    g_goal.current  = id;

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)AGENT_STATE_MONITOR << 16) | (uint32_t)id,
                    (uint32_t)prio);

    return agent_goal_planner_plan(g);
}

/* ---------- ABORT ---------- */
agent_status_t agent_goal_abort(agent_goal_id_t id)
{
    int32_t idx = find_goal_index(id);
    if (idx < 0) {
        return AGENT_ERR;
    }

    agent_goal_t *g = &g_goal.active[idx];

    g->state  = AGENT_GOAL_STATE_ABORTED;
    g->active = false;
    g_goal.aborted++;
    g_goal.active_count--;

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)id << 16) | 0xAB01U,
                    g_goal.aborted);

    return AGENT_OK;
}

/* ---------- COMPLETE ---------- */
agent_status_t agent_goal_complete(agent_goal_id_t id)
{
    int32_t idx = find_goal_index(id);
    if (idx < 0) {
        return AGENT_ERR;
    }

    agent_goal_t *g = &g_goal.active[idx];

    g->state  = AGENT_GOAL_STATE_DONE;
    g->active = false;
    g_goal.completed++;
    g_goal.active_count--;

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)id << 16) | 0xD001U,
                    g_goal.completed);

    return AGENT_OK;
}

/* ---------- UPDATE ---------- */
agent_status_t agent_goal_update(void)
{
    if (!g_goal_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    for (uint32_t i = 0U; i < AGENT_GOAL_MAX_ACTIVE; i++) {

        agent_goal_t *g = &g_goal.active[i];
        if (!g->active) {
            continue;
        }

        switch (g->state) {
        case AGENT_GOAL_STATE_PLANNING:
            (void)agent_goal_planner_step(g);
            break;

        case AGENT_GOAL_STATE_EXECUTING:
            (void)agent_goal_executor_step(g);
            break;

        case AGENT_GOAL_STATE_MONITORING:
            (void)agent_goal_monitor_step(g);
            break;

        case AGENT_GOAL_STATE_DONE:
            (void)agent_goal_complete(g->id);
            break;

        case AGENT_GOAL_STATE_FAILED:
            g->active = false;
            g_goal.failed++;
            g_goal.active_count--;
            break;

        default:
            break;
        }
    }

    g_goal.last_update_us = agent_time_now_us();
    return AGENT_OK;
}

/* ---------- QUERIES ---------- */
agent_goal_id_t agent_goal_current(void)
{
    return g_goal.current;
}

agent_goal_state_t agent_goal_state(agent_goal_id_t id)
{
    int32_t idx = find_goal_index(id);
    if (idx < 0) {
        return AGENT_GOAL_STATE_IDLE;
    }
    return g_goal.active[idx].state;
}

const agent_goal_t *agent_goal_get(agent_goal_id_t id)
{
    int32_t idx = find_goal_index(id);
    if (idx < 0) {
        return NULL;
    }
    return &g_goal.active[idx];
}

const agent_goal_state_ctx_t *agent_goal_ctx(void)
{
    return &g_goal;
}