#include "agent_modes_scheduler.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT MODES SCHEDULER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per call
 * ============================================================ */

/* ---------- Min dwell per mode (Flash-resident) ---------- */
static const uint32_t g_min_dwell_us[AGENT_MODE_ID_COUNT] = {
    0U,      /* NONE       */
    1000U,   /* NOMINAL    */
    500U,    /* DEGRADED   */
    500U,    /* VIRTUAL    */
    500U,    /* FUSION     */
    200U,    /* RECOVERY   */
    0U,      /* EMERGENCY  */
    0U       /* LANDING    */
};

/* ---------- Period per mode (µs) ---------- */
static const uint32_t g_period_us[AGENT_MODE_ID_COUNT] = {
    AGENT_MAIN_LOOP_PERIOD_US,   /* NONE       */
    AGENT_MAIN_LOOP_PERIOD_US,   /* NOMINAL    */
    1000U,                       /* DEGRADED   */
    1000U,                       /* VIRTUAL    */
    1000U,                       /* FUSION     */
    500U,                        /* RECOVERY   */
    500U,                        /* EMERGENCY  */
    500U                         /* LANDING    */
};

static bool g_sched_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_modes_scheduler_init(void)
{
    g_sched_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_modes_scheduler_reset(void)
{
    return agent_modes_scheduler_init();
}

/* ---------- ALLOW SWITCH ---------- */
bool agent_modes_scheduler_allow_switch(agent_mode_id_t from,
                                        agent_mode_id_t to,
                                        uint32_t        dwell_us)
{
    if (!g_sched_initialized)       return false;
    if (to >= AGENT_MODE_ID_COUNT)  return false;

    /* EMERGENCY always allowed */
    if (to == AGENT_MODE_ID_EMERGENCY) {
        return true;
    }

    if (from >= AGENT_MODE_ID_COUNT) {
        return true;
    }

    uint32_t min_dwell = g_min_dwell_us[from];

    if (min_dwell == 0U) {
        return true;
    }

    if (dwell_us < min_dwell) {
        agent_log_event(AGENT_LOG_TRANSITION_BLOCKED,
                        ((uint32_t)from << 16) | (uint32_t)to,
                        dwell_us);
        return false;
    }

    return true;
}

/* ---------- NEXT PERIOD ---------- */
uint32_t agent_modes_scheduler_next_period_us(agent_mode_id_t id)
{
    if (id >= AGENT_MODE_ID_COUNT) {
        return AGENT_MAIN_LOOP_PERIOD_US;
    }
    return g_period_us[id];
}

/* ---------- SLOT COUNT ----------
 * Returns total slot count = AGENT_MODE_ID_COUNT (8).
 * Includes NONE slot; active mode slots = 7.
 */
uint32_t agent_modes_scheduler_slot_count(void)
{
    return AGENT_MODE_ID_COUNT;
}