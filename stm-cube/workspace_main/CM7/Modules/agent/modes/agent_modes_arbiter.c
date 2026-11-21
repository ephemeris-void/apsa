#include "agent_modes_arbiter.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT MODES ARBITER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per select
 * ============================================================ */

static agent_mode_id_t g_forced_mode    = AGENT_MODE_ID_NONE;
static bool            g_arb_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_modes_arbiter_init(void)
{
    g_forced_mode     = AGENT_MODE_ID_NONE;
    g_arb_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_modes_arbiter_reset(void)
{
    return agent_modes_arbiter_init();
}

/* ---------- FORCE ---------- */
agent_status_t agent_modes_arbiter_force(agent_mode_id_t id)
{
    if (!g_arb_initialized)        return AGENT_ERR_NOT_INIT;
    if (id >= AGENT_MODE_ID_COUNT) return AGENT_ERR;

    g_forced_mode = id;
    return AGENT_OK;
}

/* ---------- SELECT (deterministic priority ladder) ---------- */
agent_mode_id_t agent_modes_arbiter_select(void)
{
    if (!g_arb_initialized) {
        return AGENT_MODE_ID_NONE;
    }

    /* Force-once override */
    if (g_forced_mode != AGENT_MODE_ID_NONE) {
        agent_mode_id_t f = g_forced_mode;
        g_forced_mode = AGENT_MODE_ID_NONE;
        return f;
    }

    agent_context_full_t *ctx = agent_context_get();

    /* 1. Safety envelope violated → EMERGENCY */
    if (!agent_safety_check()) {
        return AGENT_MODE_ID_EMERGENCY;
    }

    /* 2. System failure → EMERGENCY */
    if (ctx->system_health == AGENT_HEALTH_FAIL) {
        return AGENT_MODE_ID_EMERGENCY;
    }

    /* 3. Landable → LANDING */
    if (agent_safety_can_land()) {
        return AGENT_MODE_ID_LANDING;
    }

    /* 4. AI failed */
    if (ctx->ai_health == AGENT_HEALTH_FAIL) {
        if (ctx->imu_health == AGENT_HEALTH_FAIL) {
            return AGENT_MODE_ID_EMERGENCY;
        }
        return AGENT_MODE_ID_DEGRADED;
    }

    /* 5. IMU failed → VIRTUAL (if AI OK) or EMERGENCY */
    if (imu_health() == AGENT_HEALTH_FAIL) {
        if (ctx->ai_health == AGENT_HEALTH_OK) {
            return AGENT_MODE_ID_VIRTUAL;
        }
        return AGENT_MODE_ID_EMERGENCY;
    }

    /* 6. IMU degraded → DEGRADED */
    if (imu_health() == AGENT_HEALTH_DEGRADED) {
        return AGENT_MODE_ID_DEGRADED;
    }

    /* 7. Confidence dropped → VIRTUAL */
    if (ctx->confidence < AGENT_CONFIDENCE_MIN) {
        return AGENT_MODE_ID_VIRTUAL;
    }

    /* 8. State-driven selection */
    if (ctx->current_state == AGENT_STATE_RECOVERY) {
        return AGENT_MODE_ID_RECOVERY;
    }

    if (ctx->current_state == AGENT_STATE_FUSION) {
        return AGENT_MODE_ID_FUSION;
    }

    /* 9. Default nominal */
    return AGENT_MODE_ID_NOMINAL;
}