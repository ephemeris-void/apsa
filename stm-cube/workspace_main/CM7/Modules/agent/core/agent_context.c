#include "agent_context.h"
#include "agent_config.h"
#include "agent.h"
#include "../trace/agent_log.h"

/* HAL_GetTick external declaration — avoid full HAL include */
extern uint32_t HAL_GetTick(void);

/* ============================================================
 * AGENT CONTEXT — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Single global instance. No dynamic allocation.
 *
 * NOTE: agent_confidence_set()/get() live in agent_confidence.c.
 *       Do NOT define them here — linker conflict.
 * ============================================================ */

/* ---------- DWT Cycle Counter ---------- */
#define DWT_CYCCNT_REG    (*(volatile uint32_t *)0xE0001004UL)

/* PERF: 480 cycles per µs @ 480 MHz → reciprocal multiply */
AGENT_INLINE_FAST uint32_t dwt_now_us(void)
{
    return (uint32_t)((uint64_t)HAL_GetTick() * 1000ULL);
}

/* ---------- Global Instance ---------- */
static AGENT_DTCM agent_context_full_t g_ctx;

/* ---------- GET ---------- */
agent_context_full_t* agent_context_get(void)
{
    return &g_ctx;
}

/* ---------- RESET ---------- */
void agent_context_reset(void)
{
    uint32_t *p = (uint32_t *)&g_ctx;
    uint32_t  n = sizeof(g_ctx) / sizeof(uint32_t);

    for (uint32_t i = 0; i < n; i++) {
        p[i] = 0U;
    }

    g_ctx.current_state    = AGENT_STATE_BOOT;
    g_ctx.previous_state   = AGENT_STATE_BOOT;
    g_ctx.system_health    = AGENT_HEALTH_UNKNOWN;
    g_ctx.imu_health       = AGENT_HEALTH_UNKNOWN;
    g_ctx.baro_health      = AGENT_HEALTH_UNKNOWN;
    g_ctx.gps_health       = AGENT_HEALTH_UNKNOWN;
    g_ctx.ai_health        = AGENT_HEALTH_UNKNOWN;
    g_ctx.confidence       = 1.0f;
    g_ctx.confidence_min   = 1.0f;
    g_ctx.confidence_max   = 0.0f;
    g_ctx.current_priority = AGENT_PRIO_LOW;
}

/* ---------- INIT ---------- */
void agent_context_init(void)
{
    agent_context_reset();

    g_ctx.boot_time_us   = dwt_now_us();
    g_ctx.state_entry_us = g_ctx.boot_time_us;
}

/* ---------- ITERATION MARKERS ---------- */
void agent_iteration_begin(void)
{
    g_ctx.loop_start_us = dwt_now_us();
    g_ctx.iteration_count++;
}

void agent_iteration_end(void)
{
    g_ctx.loop_end_us = dwt_now_us();

    uint32_t dt = g_ctx.loop_end_us - g_ctx.loop_start_us;
    g_ctx.loop_wcet_us = dt;

    if (dt > g_ctx.loop_wcet_max_us) {
        g_ctx.loop_wcet_max_us = dt;
    }

    g_ctx.time_in_state_us = g_ctx.loop_end_us - g_ctx.state_entry_us;
}

/* ---------- HEALTH ---------- */
void agent_health_set_imu(agent_health_t h)
{
    g_ctx.imu_health = h;

    if (h == AGENT_HEALTH_FAIL || g_ctx.ai_health == AGENT_HEALTH_FAIL) {
        g_ctx.system_health = AGENT_HEALTH_FAIL;
    } else if (h == AGENT_HEALTH_DEGRADED) {
        g_ctx.system_health = AGENT_HEALTH_DEGRADED;
    } else {
        g_ctx.system_health = AGENT_HEALTH_OK;
    }
}

void agent_health_set_baro(agent_health_t h) { g_ctx.baro_health = h; }
void agent_health_set_gps(agent_health_t h)  { g_ctx.gps_health  = h; }

void agent_health_set_ai(agent_health_t h)
{
    g_ctx.ai_health = h;

    if (h == AGENT_HEALTH_FAIL || g_ctx.imu_health == AGENT_HEALTH_FAIL) {
        g_ctx.system_health = AGENT_HEALTH_FAIL;
    } else if (h == AGENT_HEALTH_DEGRADED) {
        g_ctx.system_health = AGENT_HEALTH_DEGRADED;
    } else {
        g_ctx.system_health = AGENT_HEALTH_OK;
    }
}

agent_health_t agent_health_get_system(void)
{
    return g_ctx.system_health;
}

/* ---------- CONFIDENCE (removed — see agent_confidence.c) ----------
 * agent_confidence_set()/get() are defined in agent_confidence.c.
 * Context->confidence is updated by agent_confidence_set().
 */

/* ---------- FAULT ---------- */
void agent_fault_register(agent_fault_t code)
{
    if (g_ctx.fault_count < 0xFFFFFFFFUL) {
        g_ctx.fault_count++;
    }
    if (g_ctx.fault_consecutive < 0xFFFFFFFFUL) {
        g_ctx.fault_consecutive++;
    }
    g_ctx.fault_last_code = (uint32_t)code;
    g_ctx.fault_last_us   = dwt_now_us();

    agent_log_event(AGENT_LOG_DEADLINE_MISS, (uint32_t)code, g_ctx.fault_count);
}

uint32_t agent_fault_get_count(void) { return g_ctx.fault_count; }
uint32_t agent_fault_get_last(void)  { return g_ctx.fault_last_code; }

/* ---------- RECOVERY ---------- */
void agent_recovery_register(void)
{
    g_ctx.recovery_count++;
    g_ctx.recovery_attempts = 0U;
    g_ctx.fault_consecutive = 0U;
}

uint32_t agent_recovery_get_attempts(void) { return g_ctx.recovery_attempts; }

/* ---------- TIMING ---------- */
uint32_t agent_time_now_us(void)
{
    return dwt_now_us();
}

uint32_t agent_time_in_state_us(void)
{
    return dwt_now_us() - g_ctx.state_entry_us;
}

uint32_t agent_time_since_boot_us(void)
{
    return dwt_now_us() - g_ctx.boot_time_us;
}

/* ---------- DEADLINE CHECK ---------- */
bool agent_deadline_check(void)
{
    uint32_t now = dwt_now_us();
    uint32_t elapsed = now - g_ctx.loop_start_us;

    if (elapsed > AGENT_DEADLINE_US) {
        g_ctx.deadline_miss_count++;
        return false;
    }
    return true;
}

/* ---------- WCET LOOP CHECK ---------- */
bool agent_wcet_loop_check(void)
{
    uint32_t budget_us = AGENT_WCET_CYCLES_LOOP / 480U;

    if (g_ctx.loop_wcet_us > budget_us) {
        g_ctx.wcet_violate_count++;
        return false;
    }
    return true;
}