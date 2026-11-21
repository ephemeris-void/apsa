#include "agent_recovery_strategy.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_control_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT RECOVERY STRATEGY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs per apply
 * ============================================================ */

static agent_recovery_strategy_t g_active = AGENT_RECOVERY_STRATEGY_NONE;

/* ---------- INIT ---------- */
agent_status_t agent_recovery_strategy_init(void)
{
    g_active = AGENT_RECOVERY_STRATEGY_NONE;
    return AGENT_OK;
}

/* ---------- Strategy: RESTART (all subsystems) ---------- */
static agent_status_t strategy_restart(void)
{
    (void)imu_init();
    (void)ai_init();
    (void)ekf_init();
    (void)control_init();
    return AGENT_OK;
}

/* ---------- Strategy: RESET (EKF only) ---------- */
static agent_status_t strategy_reset(void)
{
    return ekf_init();
}

/* ---------- Strategy: RECONFIG (AI only) ---------- */
static agent_status_t strategy_reconfig(void)
{
    return ai_init();
}

/* ---------- Strategy: ISOLATE (mark IMU failed) ---------- */
static agent_status_t strategy_isolate(void)
{
    agent_health_set_imu(AGENT_HEALTH_FAIL);
    return AGENT_OK;
}

/* ---------- Strategy: DEGRADE (mark IMU degraded) ---------- */
static agent_status_t strategy_degrade(void)
{
    agent_health_set_imu(AGENT_HEALTH_DEGRADED);
    return AGENT_OK;
}

/* ---------- APPLY ---------- */
agent_status_t agent_recovery_strategy_apply(agent_recovery_strategy_t strategy)
{
    g_active = strategy;

    switch (strategy) {
    case AGENT_RECOVERY_STRATEGY_RESTART:  return strategy_restart();
    case AGENT_RECOVERY_STRATEGY_RESET:    return strategy_reset();
    case AGENT_RECOVERY_STRATEGY_RECONFIG: return strategy_reconfig();
    case AGENT_RECOVERY_STRATEGY_ISOLATE:  return strategy_isolate();
    case AGENT_RECOVERY_STRATEGY_DEGRADE:  return strategy_degrade();
    default:
        return AGENT_ERR;
    }
}

/* ---------- STATUS ---------- */
agent_recovery_strategy_t agent_recovery_strategy_status(void)
{
    return g_active;
}

/* ---------- CLEAR ---------- */
void agent_recovery_strategy_clear(void)
{
    g_active = AGENT_RECOVERY_STRATEGY_NONE;
}