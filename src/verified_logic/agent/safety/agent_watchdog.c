#include "agent_watchdog.h"
#include "agent_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT WATCHDOG — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs
 * ============================================================ */

static bool     g_wdt_initialized  = false;
static uint32_t g_wdt_last_kick_us = 0U;
static uint32_t g_wdt_kick_count   = 0U;

/* ---------- INIT ---------- */
agent_status_t agent_watchdog_init(void)
{
    g_wdt_last_kick_us = 0U;
    g_wdt_kick_count   = 0U;
    g_wdt_initialized  = true;

    agent_log_event(AGENT_LOG_WDT_INIT, 0U, 0U);
    return AGENT_OK;
}

/* ---------- KICK ---------- */
void agent_watchdog_kick(void)
{
    if (!g_wdt_initialized) {
        return;
    }

    g_wdt_last_kick_us = agent_time_now_us();
    g_wdt_kick_count++;

    agent_context_full_t *ctx = agent_context_get();
    ctx->wdt_last_kick_us = g_wdt_last_kick_us;
    ctx->wdt_kick_count   = g_wdt_kick_count;
}

/* ---------- CHECK ---------- */
bool agent_watchdog_check(void)
{
    if (!g_wdt_initialized) {
        return false;
    }

    uint32_t now     = agent_time_now_us();
    uint32_t elapsed = now - g_wdt_last_kick_us;

    if (elapsed > AGENT_WDT_TIMEOUT_US) {
        agent_log_event(AGENT_LOG_WDT_TIMEOUT, elapsed, 0U);
        agent_fault_register(AGENT_FAULT_WATCHDOG);
        agent_safety_escalate();
        return false;
    }

    return true;
}

/* ---------- QUERIES ---------- */
uint32_t agent_watchdog_last_kick_us(void)
{
    return g_wdt_last_kick_us;
}

uint32_t agent_watchdog_kick_count(void)
{
    return g_wdt_kick_count;
}

/* ---------- RESET ---------- */
void agent_watchdog_reset(void)
{
    g_wdt_last_kick_us = agent_time_now_us();
    g_wdt_kick_count   = 0U;
}