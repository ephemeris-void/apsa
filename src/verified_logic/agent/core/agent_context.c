#include "agent_context.h"
#include "agent_config.h"
#include "agent.h"
#include "../trace/agent_log.h"

#include <zephyr/kernel.h>

#define DWT_CYCCNT_REG    (*(volatile uint32_t *)0xE0001004UL)

/* DWT cycle counter → µs. Used for timing (WCET, loop, deadline). */
AGENT_INLINE_FAST uint32_t dwt_now_us(void)
{
    return (uint32_t)((float)DWT_CYCCNT_REG * AGENT_RECIP_480);
}

/* Zephyr uptime in µs. Wraps every ~71 minutes. Used for the
 * virtual-only accumulator so it is immune to DWT 8.95 s wraps. */
AGENT_INLINE_FAST uint32_t uptime_now_us(void)
{
    return (uint32_t)(k_uptime_get() * 1000ULL);
}

static AGENT_DTCM agent_context_full_t g_ctx;

agent_context_full_t* agent_context_get(void) { return &g_ctx; }

void agent_context_reset(void)
{
    uint32_t *p = (uint32_t *)&g_ctx;
    uint32_t  n = sizeof(g_ctx) / sizeof(uint32_t);
    for (uint32_t i = 0; i < n; i++) p[i] = 0U;

    g_ctx.current_state    = AGENT_STATE_BOOT;
    g_ctx.previous_state   = AGENT_STATE_BOOT;
    g_ctx.current_priority = AGENT_PRIO_LOW;

    g_ctx.imu_health        = AGENT_HEALTH_UNKNOWN;
    g_ctx.ai_health         = AGENT_HEALTH_UNKNOWN;
    g_ctx.system_health     = AGENT_HEALTH_UNKNOWN;

    g_ctx.imu_fail_since_us = 0U;
    g_ctx.imu_fail_accum_us = 0U;
    g_ctx.imu_fail_last_us  = 0U;
}

void agent_context_init(void)
{
    agent_context_reset();
    g_ctx.boot_time_us   = uptime_now_us();
    g_ctx.state_entry_us = g_ctx.boot_time_us;
}

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
    if (dt > g_ctx.loop_wcet_max_us) g_ctx.loop_wcet_max_us = dt;

    /* time_in_state: use uptime (no wrap) so it never goes 0 spuriously */
    uint32_t up = uptime_now_us();
    g_ctx.time_in_state_us = up - g_ctx.state_entry_us;
}

void agent_health_set_imu(agent_health_t h)
{
    agent_health_t prev = g_ctx.imu_health;
    g_ctx.imu_health = h;

    if (h == AGENT_HEALTH_FAIL) {
        if (prev != AGENT_HEALTH_FAIL) {
            uint32_t now = uptime_now_us();
            g_ctx.imu_fail_since_us = now;
            g_ctx.imu_fail_accum_us = 0U;
            g_ctx.imu_fail_last_us  = now;
        } else {
            uint32_t now = uptime_now_us();
            uint32_t delta = now - g_ctx.imu_fail_last_us;
            if (delta > 10000U) delta = 1000U;   /* scheduler hiccup guard */
            g_ctx.imu_fail_accum_us += delta;
            g_ctx.imu_fail_last_us   = now;
        }
    } else if (h == AGENT_HEALTH_OK) {
        g_ctx.imu_fail_since_us = 0U;
        g_ctx.imu_fail_accum_us = 0U;
        g_ctx.imu_fail_last_us  = 0U;
    }
}

void agent_health_set_ai(agent_health_t h) { g_ctx.ai_health = h; }

void agent_health_refresh(void)
{
    agent_health_t i = g_ctx.imu_health;
    agent_health_t a = g_ctx.ai_health;

    if (i == AGENT_HEALTH_FAIL || a == AGENT_HEALTH_FAIL) {
        g_ctx.system_health = AGENT_HEALTH_FAIL;
    } else if (i == AGENT_HEALTH_DEGRADED || a == AGENT_HEALTH_DEGRADED) {
        g_ctx.system_health = AGENT_HEALTH_DEGRADED;
    } else if (i == AGENT_HEALTH_OK && a == AGENT_HEALTH_OK) {
        g_ctx.system_health = AGENT_HEALTH_OK;
    } else {
        g_ctx.system_health = AGENT_HEALTH_UNKNOWN;
    }
}

agent_health_t agent_health_get_system(void) { return g_ctx.system_health; }

void agent_fault_register(agent_fault_t code)
{
    g_ctx.fault_count++;
    g_ctx.fault_consecutive++;
    g_ctx.fault_last_code = (uint32_t)code;
    g_ctx.fault_last_us   = uptime_now_us();
    agent_log_event(AGENT_LOG_DEADLINE_MISS, (uint32_t)code, g_ctx.fault_count);
}

uint32_t agent_fault_get_count(void) { return g_ctx.fault_count; }
uint32_t agent_fault_get_last(void)  { return g_ctx.fault_last_code; }

void agent_recovery_register(void)
{
    g_ctx.recovery_count++;
    g_ctx.recovery_attempts = 0U;
    g_ctx.fault_consecutive = 0U;
}

uint32_t agent_recovery_get_attempts(void) { return g_ctx.recovery_attempts; }

uint32_t agent_time_now_us(void) { return uptime_now_us(); }

uint32_t agent_time_in_state_us(void)
{
    uint32_t now = uptime_now_us();
    return now - g_ctx.state_entry_us;
}

uint32_t agent_time_since_boot_us(void) { return uptime_now_us() - g_ctx.boot_time_us; }

bool agent_deadline_check(void)
{
    uint32_t now = dwt_now_us();
    uint32_t elapsed = now - g_ctx.loop_start_us;
    if (elapsed > (AGENT_DEADLINE_US * 3U)) {
        g_ctx.deadline_miss_count++;
        return false;
    }
    return true;
}

bool agent_wcet_loop_check(void)
{
    uint32_t now = dwt_now_us();
    uint32_t elapsed = now - g_ctx.loop_start_us;
    if (elapsed > (AGENT_DEADLINE_US * 3U)) {
        g_ctx.wcet_violate_count++;
        return false;
    }
    return true;
}