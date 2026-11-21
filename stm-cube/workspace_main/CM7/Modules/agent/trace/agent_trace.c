#include "stm32h7xx.h"
#include "agent_trace.h"
#include "agent_log.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"

/* ============================================================
 * AGENT TRACE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Counter: DWT->CYCCNT (free-running, 32-bit)
 * WCET: 5 µs per trace call
 * ============================================================ */

/* PERF: direct register access instead of DWT->CYCCNT deref */
#define DWT_CYCCNT_REG    (*(volatile uint32_t *)0xE0001004UL)

/* ---------- Per-Function Cycle Snapshots ---------- */
static uint32_t g_t_loop_begin;
static uint32_t g_t_state_begin;
static uint32_t g_t_decision_begin;
static uint32_t g_t_action_begin;
static uint32_t g_t_safety_begin;
static uint32_t g_t_ai_begin;

/* ---------- Accumulated Stats ---------- */
typedef struct {
    uint32_t count;
    uint32_t cycles_min;
    uint32_t cycles_max;
    uint32_t cycles_sum;
    uint32_t deadline_cycles;
    uint32_t miss_count;
} trace_stat_t;

static trace_stat_t g_stat_loop     = {0U, 0xFFFFFFFFU, 0U, 0U, AGENT_WCET_CYCLES_LOOP,   0U};
static trace_stat_t g_stat_decision = {0U, 0xFFFFFFFFU, 0U, 0U, AGENT_WCET_CYCLES_DECIDE, 0U};
static trace_stat_t g_stat_action   = {0U, 0xFFFFFFFFU, 0U, 0U, AGENT_WCET_CYCLES_ACT,    0U};
static trace_stat_t g_stat_safety   = {0U, 0xFFFFFFFFU, 0U, 0U, 48000U, 0U};   /* 100 µs */
static trace_stat_t g_stat_ai       = {0U, 0xFFFFFFFFU, 0U, 0U, AGENT_WCET_CYCLES_AI,     0U};

/* ---------- Internal stat update ---------- */
AGENT_INLINE_FAST void trace_stat_update(trace_stat_t *s, uint32_t cycles)
{
    s->count++;
    if (cycles < s->cycles_min) s->cycles_min = cycles;
    if (cycles > s->cycles_max) s->cycles_max = cycles;
    s->cycles_sum += cycles;
    if (cycles > s->deadline_cycles) s->miss_count++;
}

/* ---------- INIT ---------- */
agent_status_t agent_trace_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_CYCCNT_REG = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    agent_log_event(AGENT_LOG_TRACE_INIT, 0U, 0U);
    return AGENT_OK;
}

/* ---------- CYCLE API ---------- */
uint32_t agent_trace_now_cycles(void)
{
    return DWT_CYCCNT_REG;
}

uint32_t agent_trace_cycles_to_us(uint32_t cycles)
{
    /* PERF: reciprocal multiply instead of integer division
     *       (~20 cycles → ~1 cycle) */
    return (uint32_t)((float)cycles * AGENT_RECIP_480);
}

/* ---------- LOOP ---------- */
void agent_trace_loop_begin(void)
{
    g_t_loop_begin = DWT_CYCCNT_REG;
}

void agent_trace_loop_end(void)
{
    uint32_t cycles = DWT_CYCCNT_REG - g_t_loop_begin;
    trace_stat_update(&g_stat_loop, cycles);

    agent_context_full_t *ctx = agent_context_get();
    ctx->loop_wcet_us = agent_trace_cycles_to_us(cycles);
    if (ctx->loop_wcet_us > ctx->loop_wcet_max_us) {
        ctx->loop_wcet_max_us = ctx->loop_wcet_us;
    }
}

/* ---------- STATE ---------- */
void agent_trace_state_enter(agent_state_t state)
{
    (void)state;    /* Reserved for future per-state histogram */
}

void agent_trace_state_begin(agent_state_t state)
{
    (void)state;
    g_t_state_begin = DWT_CYCCNT_REG;
}

void agent_trace_state_end(agent_state_t state)
{
    (void)state;
    (void)(DWT_CYCCNT_REG - g_t_state_begin);   /* Reserved */
}

/* ---------- DECISION ---------- */
void agent_trace_decision_begin(void)
{
    g_t_decision_begin = DWT_CYCCNT_REG;
}

void agent_trace_decision_end(void)
{
    uint32_t cycles = DWT_CYCCNT_REG - g_t_decision_begin;
    trace_stat_update(&g_stat_decision, cycles);
}

/* ---------- ACTION ---------- */
void agent_trace_action_begin(uint32_t action_id)
{
    (void)action_id;
    g_t_action_begin = DWT_CYCCNT_REG;
}

void agent_trace_action_end(uint32_t action_id)
{
    (void)action_id;
    uint32_t cycles = DWT_CYCCNT_REG - g_t_action_begin;
    trace_stat_update(&g_stat_action, cycles);
}

/* ---------- SAFETY ---------- */
void agent_trace_safety_begin(void)
{
    g_t_safety_begin = DWT_CYCCNT_REG;
}

void agent_trace_safety_end(void)
{
    uint32_t cycles = DWT_CYCCNT_REG - g_t_safety_begin;
    trace_stat_update(&g_stat_safety, cycles);
}

/* ---------- TRANSITION ---------- */
void agent_trace_transition(agent_state_t from,
                             agent_state_t to,
                             uint32_t      reason)
{
    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)from << 16) | (uint32_t)to,
                    reason);
}

/* ---------- AI ---------- */
void agent_trace_ai_begin(void)
{
    g_t_ai_begin = DWT_CYCCNT_REG;
}

void agent_trace_ai_end(void)
{
    uint32_t cycles = DWT_CYCCNT_REG - g_t_ai_begin;
    trace_stat_update(&g_stat_ai, cycles);

    agent_context_full_t *ctx = agent_context_get();
    ctx->ai_inference_latency_us = agent_trace_cycles_to_us(cycles);
    if (ctx->ai_inference_latency_us > ctx->ai_inference_latency_max_us) {
        ctx->ai_inference_latency_max_us = ctx->ai_inference_latency_us;
    }
}

/* ---------- DUMP STATS ---------- */
void agent_trace_dump_stats(void)
{
    agent_log_event(AGENT_LOG_TRACE_DUMP, 0U, 0U);
    /* UART dump handled by log module */
}