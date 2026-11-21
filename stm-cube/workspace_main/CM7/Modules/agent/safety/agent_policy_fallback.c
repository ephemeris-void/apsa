#include "agent_policy_fallback.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY FALLBACK — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per call
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_fb_state_t g_fb;
static bool                               g_fb_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_fallback_init(void)
{
    memset(&g_fb, 0, sizeof(g_fb));

    for (uint32_t i = 0U; i < AGENT_POLICY_FB_MAX_LEVELS; i++) {
        g_fb.entries[i].level          = (agent_policy_fb_level_t)i;
        g_fb.entries[i].policy_id      = AGENT_POLICY_ID_NONE;
        g_fb.entries[i].switch_count   = 0U;
        g_fb.entries[i].fail_count     = 0U;
        g_fb.entries[i].last_switch_us = 0U;
    }

    g_fb.active_level   = AGENT_POLICY_FB_LEVEL_PRIMARY;
    g_fb.total_switches = 0U;
    g_fb.total_fails    = 0U;
    g_fb.last_update_us = agent_time_now_us();

    g_fb_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_fallback_reset(void)
{
    g_fb_initialized = false;
    return agent_policy_fallback_init();
}

/* ---------- REGISTER ---------- */
agent_status_t agent_policy_fallback_register(agent_policy_fb_level_t level,
                                              agent_policy_id_t       policy_id)
{
    if (!g_fb_initialized)                          return AGENT_ERR_NOT_INIT;
    if (level >= AGENT_POLICY_FB_MAX_LEVELS)        return AGENT_ERR;
    if (policy_id >= AGENT_POLICY_ID_COUNT)         return AGENT_ERR;

    g_fb.entries[level].policy_id = policy_id;
    return AGENT_OK;
}

/* ---------- NEXT ---------- */
agent_policy_fb_level_t agent_policy_fallback_next(agent_policy_fb_level_t current)
{
    if (current >= (AGENT_POLICY_FB_MAX_LEVELS - 1U)) {
        return AGENT_POLICY_FB_LEVEL_SAFE;
    }
    return (agent_policy_fb_level_t)(current + 1U);
}

/* ---------- SWITCH ---------- */
agent_status_t agent_policy_fallback_switch(agent_policy_fb_level_t level)
{
    if (!g_fb_initialized)                      return AGENT_ERR_NOT_INIT;
    if (level >= AGENT_POLICY_FB_MAX_LEVELS)    return AGENT_ERR;

    if (level == g_fb.active_level) {
        return AGENT_OK;
    }

    agent_policy_fb_level_t prev = g_fb.active_level;

    g_fb.active_level = level;
    g_fb.entries[level].switch_count++;
    g_fb.entries[level].last_switch_us = agent_time_now_us();
    g_fb.total_switches++;
    g_fb.last_update_us = agent_time_now_us();

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)prev << 16) | (uint32_t)level,
                    0x01U);

    return AGENT_OK;
}

/* ---------- ACTIVE ---------- */
agent_policy_fb_level_t agent_policy_fallback_active(void)
{
    return g_fb.active_level;
}

/* ---------- QUERIES ---------- */
const agent_policy_fb_entry_t *agent_policy_fallback_entry(agent_policy_fb_level_t level)
{
    if (level >= AGENT_POLICY_FB_MAX_LEVELS) {
        return NULL;
    }
    return &g_fb.entries[level];
}

const agent_policy_fb_state_t *agent_policy_fallback_state(void)
{
    return &g_fb;
}