#include "agent_mission.h"
#include "agent_mission_state.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT MISSION — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 100 µs per update
 * ============================================================ */

/* PERF: context + waypoints in DTCM (zero wait state) */
static AGENT_DTCM agent_mission_ctx_t      g_mission;
static AGENT_DTCM agent_mission_waypoint_t g_waypoints[AGENT_MISSION_MAX_WAYPOINTS];
static bool                                g_mission_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_mission_init(void)
{
    memset(&g_mission, 0, sizeof(g_mission));       /* zero-init incl. reserved[] */
    memset(g_waypoints, 0, sizeof(g_waypoints));    /* zero-init incl. reserved[] */

    g_mission.state           = AGENT_MISSION_IDLE;
    g_mission.previous        = AGENT_MISSION_IDLE;
    g_mission.current_wp      = 0U;
    g_mission.wp_count        = 0U;
    g_mission.start_us        = 0U;
    g_mission.elapsed_us      = 0U;
    g_mission.state_entry_us  = agent_time_now_us();
    g_mission.loops_completed = 0U;
    g_mission.active          = false;

    agent_mission_state_init();

    g_mission_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_mission_reset(void)
{
    g_mission_initialized = false;
    return agent_mission_init();
}

/* ---------- LOAD ---------- */
agent_status_t agent_mission_load(const agent_mission_waypoint_t *wps, uint32_t count)
{
    if (wps == NULL || count == 0U)             return AGENT_ERR;
    if (count > AGENT_MISSION_MAX_WAYPOINTS)    return AGENT_ERR;

    for (uint32_t i = 0U; i < count; i++) {
        g_waypoints[i] = wps[i];
    }

    g_mission.wp_count   = count;
    g_mission.current_wp = 0U;

    return AGENT_OK;
}

/* ---------- State transition ---------- */
static agent_status_t mission_transition(agent_mission_state_t next, uint32_t reason)
{
    if (next == g_mission.state) {
        return AGENT_OK;
    }

    g_mission.previous       = g_mission.state;
    g_mission.state          = next;
    g_mission.state_entry_us = agent_time_now_us();

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)g_mission.previous << 16) | (uint32_t)next,
                    reason);

    return agent_mission_state_enter(next);
}

/* ---------- START ---------- */
agent_status_t agent_mission_start(void)
{
    if (!g_mission_initialized)     return AGENT_ERR_NOT_INIT;
    if (g_mission.wp_count == 0U)   return AGENT_ERR;

    g_mission.active          = true;
    g_mission.start_us        = agent_time_now_us();
    g_mission.elapsed_us      = 0U;
    g_mission.current_wp      = 0U;
    g_mission.loops_completed = 0U;

    return mission_transition(AGENT_MISSION_ARMED, 0x0001U);
}

/* ---------- ABORT ---------- */
agent_status_t agent_mission_abort(void)
{
    g_mission.active = false;
    return mission_transition(AGENT_MISSION_ABORTED, 0x0002U);
}

/* ---------- HOLD ---------- */
agent_status_t agent_mission_hold(void)
{
    return mission_transition(AGENT_MISSION_LOITER, 0x0003U);
}

/* ---------- RESUME ---------- */
agent_status_t agent_mission_resume(void)
{
    if (!g_mission.active) {
        return AGENT_ERR_INVALID_STATE;
    }
    return mission_transition(AGENT_MISSION_ENROUTE, 0x0004U);
}

/* ---------- ADVANCE ---------- */
agent_status_t agent_mission_advance(void)
{
    if (g_mission.current_wp + 1U >= g_mission.wp_count) {
        return mission_transition(AGENT_MISSION_COMPLETE, 0x0005U);
    }

    g_mission.current_wp++;

    agent_log_event(AGENT_LOG_TRANSITION,
                    (uint32_t)g_mission.current_wp,
                    0x0006U);

    return AGENT_OK;
}

/* ---------- UPDATE ---------- */
agent_status_t agent_mission_update(void)
{
    if (!g_mission_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    if (!g_mission.active) {
        return AGENT_OK;
    }

    uint32_t now = agent_time_now_us();
    g_mission.elapsed_us = now - g_mission.start_us;

    if (g_mission.elapsed_us > AGENT_MISSION_TIMEOUT_US) {
        (void)mission_transition(AGENT_MISSION_FAILED, 0x0101U);
        return AGENT_ERR_TIMEOUT;
    }

    return agent_mission_state_step();
}

/* ---------- QUERIES ---------- */
agent_mission_state_t agent_mission_state(void)
{
    return g_mission.state;
}

uint32_t agent_mission_current_wp(void)
{
    return g_mission.current_wp;
}

const agent_mission_waypoint_t *agent_mission_waypoint(uint32_t idx)
{
    if (idx >= g_mission.wp_count) {
        return NULL;
    }
    return &g_waypoints[idx];
}

const agent_mission_ctx_t *agent_mission_ctx(void)
{
    return &g_mission;
}