#include "agent_mission_state.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"
#include <math.h>

/* ============================================================
 * AGENT MISSION STATE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs per step
 * ============================================================ */

static bool g_state_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_mission_state_init(void)
{
    g_state_initialized = true;
    return AGENT_OK;
}

/* ---------- ENTER ---------- */
agent_status_t agent_mission_state_enter(agent_mission_state_t state)
{
    if (!g_state_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    switch (state) {
    case AGENT_MISSION_ARMED:    agent_log_event(AGENT_LOG_STATE_ENTER, 0x0201U, 0U); break;
    case AGENT_MISSION_TAKEOFF:  agent_log_event(AGENT_LOG_STATE_ENTER, 0x0202U, 0U); break;
    case AGENT_MISSION_ENROUTE:  agent_log_event(AGENT_LOG_STATE_ENTER, 0x0203U, 0U); break;
    case AGENT_MISSION_LOITER:   agent_log_event(AGENT_LOG_STATE_ENTER, 0x0204U, 0U); break;
    case AGENT_MISSION_RTL:      agent_log_event(AGENT_LOG_STATE_ENTER, 0x0205U, 0U); break;
    case AGENT_MISSION_LANDING:  agent_log_event(AGENT_LOG_STATE_ENTER, 0x0206U, 0U); break;
    case AGENT_MISSION_COMPLETE: agent_log_event(AGENT_LOG_STATE_ENTER, 0x0207U, 0U); break;
    case AGENT_MISSION_ABORTED:
    case AGENT_MISSION_FAILED:
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0x0208U, (uint32_t)state);
        break;
    default:
        break;
    }

    return AGENT_OK;
}

/* ---------- STEP: ARMED ---------- */
static agent_status_t step_armed(void)
{
    /* Ready to launch: advance to TAKEOFF on next update */
    return AGENT_OK;
}

/* ---------- STEP: TAKEOFF ---------- */
static agent_status_t step_takeoff(void)
{
    const agent_mission_waypoint_t *wp = agent_mission_waypoint(0U);
    if (wp == NULL) {
        return AGENT_ERR;
    }

    agent_ekf_state_t st;
    if (ekf_get_state(&st) != AGENT_OK) {
        return AGENT_ERR;
    }

    float dz = wp->pos[2] - st.pos[2];

    if (fabsf(dz) < 0.5f) {
        (void)agent_mission_advance();
    }
    return AGENT_OK;
}

/* ---------- STEP: ENROUTE ---------- */
static agent_status_t step_enroute(void)
{
    const agent_mission_waypoint_t *wp =
        agent_mission_waypoint(agent_mission_current_wp());

    if (wp == NULL) {
        return AGENT_ERR;
    }

    agent_ekf_state_t st;
    if (ekf_get_state(&st) != AGENT_OK) {
        return AGENT_ERR;
    }

    float dx = wp->pos[0] - st.pos[0];
    float dy = wp->pos[1] - st.pos[1];
    float dz = wp->pos[2] - st.pos[2];

    /* PERF: squared compare (no sqrt) */
    float d2 = dx*dx + dy*dy + dz*dz;

    if (d2 < 1.0f) {   /* < 1 m */
        (void)agent_mission_advance();
    }
    return AGENT_OK;
}

/* ---------- STEP: LOITER ---------- */
static agent_status_t step_loiter(void)
{
    /* Hold position — no advance */
    return AGENT_OK;
}

/* ---------- STEP: RTL ---------- */
static agent_status_t step_rtl(void)
{
    /* Return-to-launch — handled by waypoint 0 target */
    return AGENT_OK;
}

/* ---------- STEP: LANDING ---------- */
static agent_status_t step_landing(void)
{
    agent_ekf_state_t st;
    if (ekf_get_state(&st) != AGENT_OK) {
        return AGENT_ERR;
    }

    if (st.pos[2] < 0.3f) {
        (void)agent_mission_advance();
    }
    return AGENT_OK;
}

/* ---------- STEP (dispatch) ---------- */
agent_status_t agent_mission_state_step(void)
{
    if (!g_state_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    switch (agent_mission_state()) {
    case AGENT_MISSION_ARMED:   return step_armed();
    case AGENT_MISSION_TAKEOFF: return step_takeoff();
    case AGENT_MISSION_ENROUTE: return step_enroute();
    case AGENT_MISSION_LOITER:  return step_loiter();
    case AGENT_MISSION_RTL:     return step_rtl();
    case AGENT_MISSION_LANDING: return step_landing();
    default:                    return AGENT_OK;
    }
}