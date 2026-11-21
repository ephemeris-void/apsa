#include "agent_modes.h"
#include "agent_modes_arbiter.h"
#include "agent_modes_scheduler.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT MODES — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 30 µs per update
 * ============================================================ */

/* ---------- Mode Descriptor Table (Flash-resident) ---------- */
static const agent_mode_desc_t g_modes[AGENT_MODE_ID_COUNT] = {
    { AGENT_MODE_ID_NONE,      AGENT_MODE_KIND_NONE,     AGENT_STATE_BOOT,       AGENT_PRIO_LOW,      0U,    0U },
    { AGENT_MODE_ID_NOMINAL,   AGENT_MODE_KIND_NORMAL,   AGENT_STATE_MONITOR,    AGENT_PRIO_NORMAL,   0U, 1000U },
    { AGENT_MODE_ID_DEGRADED,  AGENT_MODE_KIND_DEGRADED, AGENT_STATE_DEGRADED,   AGENT_PRIO_HIGH,     0U, 1000U },
    { AGENT_MODE_ID_VIRTUAL,   AGENT_MODE_KIND_DEGRADED, AGENT_STATE_VIRTUALIZE, AGENT_PRIO_HIGH,     0U,  500U },
    { AGENT_MODE_ID_FUSION,    AGENT_MODE_KIND_NORMAL,   AGENT_STATE_FUSION,     AGENT_PRIO_HIGH,     0U,  500U },
    { AGENT_MODE_ID_RECOVERY,  AGENT_MODE_KIND_SAFETY,   AGENT_STATE_RECOVERY,   AGENT_PRIO_HIGH,     0U,  500U },
    { AGENT_MODE_ID_EMERGENCY, AGENT_MODE_KIND_CRITICAL, AGENT_STATE_EMERGENCY,  AGENT_PRIO_CRITICAL, 0U,    0U },
    { AGENT_MODE_ID_LANDING,   AGENT_MODE_KIND_CRITICAL, AGENT_STATE_LANDING,    AGENT_PRIO_CRITICAL, 0U,    0U }
};

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_mode_state_t g_mode_state;
static bool                          g_modes_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_modes_init(void)
{
    if (g_modes_initialized) {
        return AGENT_OK;
    }

    g_mode_state.active       = AGENT_MODE_ID_NOMINAL;
    g_mode_state.pending      = AGENT_MODE_ID_NONE;
    g_mode_state.previous     = AGENT_MODE_ID_NONE;
    g_mode_state.entry_us     = agent_time_now_us();
    g_mode_state.dwell_us     = 0U;
    g_mode_state.switch_count = 0U;
    g_mode_state.locked       = false;

    agent_modes_arbiter_init();
    agent_modes_scheduler_init();

    g_modes_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_modes_reset(void)
{
    g_modes_initialized = false;
    return agent_modes_init();
}

/* ---------- Descriptor ---------- */
const agent_mode_desc_t *agent_modes_desc(agent_mode_id_t id)
{
    if (id >= AGENT_MODE_ID_COUNT) {
        return NULL;
    }
    return &g_modes[id];
}

/* ---------- Active / Pending ---------- */
agent_mode_id_t agent_modes_active(void)
{
    return g_mode_state.active;
}

agent_mode_id_t agent_modes_pending(void)
{
    return g_mode_state.pending;
}

/* ---------- REQUEST ---------- */
agent_status_t agent_modes_request(agent_mode_id_t id)
{
    if (!g_modes_initialized)          return AGENT_ERR_NOT_INIT;
    if (id >= AGENT_MODE_ID_COUNT)     return AGENT_ERR;
    if (id == g_mode_state.active)     return AGENT_OK;

    /* Locked mode: only EMERGENCY can override */
    if (g_mode_state.locked && id != AGENT_MODE_ID_EMERGENCY) {
        return AGENT_ERR_INVALID_STATE;
    }

    g_mode_state.pending = id;
    return AGENT_OK;
}

/* ---------- UPDATE ---------- */
agent_status_t agent_modes_update(void)
{
    if (!g_modes_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    uint32_t now = agent_time_now_us();
    g_mode_state.dwell_us = now - g_mode_state.entry_us;

    /* Arbiter selects desired mode based on system state */
    agent_mode_id_t requested = agent_modes_arbiter_select();

    if (requested != g_mode_state.active &&
        requested != AGENT_MODE_ID_NONE) {
        (void)agent_modes_request(requested);
    }

    agent_mode_id_t next = g_mode_state.pending;

    if (next == AGENT_MODE_ID_NONE || next == g_mode_state.active) {
        return AGENT_OK;
    }

    const agent_mode_desc_t *desc = agent_modes_desc(next);
    if (desc == NULL) {
        return AGENT_ERR;
    }

    if (!agent_modes_scheduler_allow_switch(g_mode_state.active,
                                            next,
                                            g_mode_state.dwell_us)) {
        return AGENT_OK;
    }

    g_mode_state.previous     = g_mode_state.active;
    g_mode_state.active       = next;
    g_mode_state.pending      = AGENT_MODE_ID_NONE;
    g_mode_state.entry_us     = now;
    g_mode_state.dwell_us     = 0U;
    g_mode_state.switch_count++;

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)g_mode_state.previous << 16) | (uint32_t)next,
                    (uint32_t)desc->priority);

    return AGENT_OK;
}

/* ---------- STATE ---------- */
const agent_mode_state_t *agent_modes_state(void)
{
    return &g_mode_state;
}