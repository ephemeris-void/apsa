#include "agent_recovery.h"
#include "agent_recovery_strategy.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT RECOVERY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 100 µs per update
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_recovery_ctx_t g_recovery;
static bool                            g_recovery_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_recovery_init(void)
{
    memset(&g_recovery, 0, sizeof(g_recovery));

    g_recovery.state           = AGENT_RECOVERY_IDLE;
    g_recovery.strategy        = AGENT_RECOVERY_STRATEGY_NONE;
    g_recovery.attempts        = 0U;
    g_recovery.max_attempts    = AGENT_RECOVERY_MAX_ATTEMPTS;
    g_recovery.start_us        = 0U;
    g_recovery.last_attempt_us = 0U;
    g_recovery.success_count   = 0U;
    g_recovery.fail_count      = 0U;
    g_recovery.window_start_us = agent_time_now_us();

    agent_recovery_strategy_init();

    g_recovery_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_recovery_reset(void)
{
    g_recovery_initialized = false;
    return agent_recovery_init();
}

/* ---------- START ---------- */
agent_status_t agent_recovery_start(agent_recovery_strategy_t strategy)
{
    if (!g_recovery_initialized)                     return AGENT_ERR_NOT_INIT;
    if (strategy == AGENT_RECOVERY_STRATEGY_NONE)    return AGENT_ERR;

    uint32_t now = agent_time_now_us();

    /* Reset window if expired */
    if ((now - g_recovery.window_start_us) > AGENT_RECOVERY_WINDOW_US) {
        g_recovery.attempts        = 0U;
        g_recovery.window_start_us = now;
    }

    if (g_recovery.attempts >= g_recovery.max_attempts) {
        g_recovery.state = AGENT_RECOVERY_EXHAUSTED;
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0x01U, g_recovery.attempts);
        return AGENT_ERR;
    }

    g_recovery.state           = AGENT_RECOVERY_ATTEMPTING;
    g_recovery.strategy        = strategy;
    g_recovery.start_us        = now;
    g_recovery.last_attempt_us = now;
    g_recovery.attempts++;

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)AGENT_STATE_RECOVERY << 16) | (uint32_t)strategy,
                    g_recovery.attempts);

    return agent_recovery_strategy_apply(strategy);
}

/* ---------- ABORT ---------- */
agent_status_t agent_recovery_abort(void)
{
    g_recovery.state    = AGENT_RECOVERY_IDLE;
    g_recovery.strategy = AGENT_RECOVERY_STRATEGY_NONE;
    return AGENT_OK;
}

/* ---------- UPDATE ---------- */
agent_status_t agent_recovery_update(void)
{
    if (!g_recovery_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    /* Terminal states — no action */
    if (g_recovery.state == AGENT_RECOVERY_IDLE      ||
        g_recovery.state == AGENT_RECOVERY_SUCCEEDED ||
        g_recovery.state == AGENT_RECOVERY_FAILED    ||
        g_recovery.state == AGENT_RECOVERY_EXHAUSTED) {
        return AGENT_OK;
    }

    uint32_t now = agent_time_now_us();

    /* ATTEMPTING: wait for strategy completion */
    if (g_recovery.state == AGENT_RECOVERY_ATTEMPTING) {
        agent_recovery_strategy_t st = agent_recovery_strategy_status();

        if (st == AGENT_RECOVERY_STRATEGY_NONE) {
            g_recovery.state = AGENT_RECOVERY_VERIFYING;
        }
        return AGENT_OK;
    }

    /* VERIFYING: check if health recovered */
    if (g_recovery.state == AGENT_RECOVERY_VERIFYING) {

        if ((now - g_recovery.start_us) < AGENT_RECOVERY_MIN_DWELL_US) {
            return AGENT_OK;
        }

        agent_health_t h = agent_health_get_system();

        if (h == AGENT_HEALTH_OK) {
            g_recovery.state = AGENT_RECOVERY_SUCCEEDED;
            g_recovery.success_count++;
            agent_recovery_register();
            agent_log_event(AGENT_LOG_SENSOR_RECOVERED, g_recovery.attempts, 0U);
        } else if (g_recovery.attempts >= g_recovery.max_attempts) {
            g_recovery.state = AGENT_RECOVERY_EXHAUSTED;
            g_recovery.fail_count++;
            agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0x02U, g_recovery.attempts);
        } else {
            g_recovery.state = AGENT_RECOVERY_FAILED;
            g_recovery.fail_count++;
        }
    }

    return AGENT_OK;
}

/* ---------- QUERIES ---------- */
agent_recovery_state_t agent_recovery_state(void)
{
    return g_recovery.state;
}

agent_recovery_strategy_t agent_recovery_strategy(void)
{
    return g_recovery.strategy;
}

const agent_recovery_ctx_t *agent_recovery_ctx(void)
{
    return &g_recovery;
}