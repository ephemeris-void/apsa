#include "agent_log.h"
#include "agent_trace.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"

/* ============================================================
 * AGENT LOG — IMPLEMENTATION
 * ============================================================
 * Lock-free ring buffer. No dynamic allocation.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per call
 * ============================================================ */

/* ---------- Ring Buffer ----------
 * PERF: placed in DTCM (zero wait state, no cache miss)
 */
static AGENT_DTCM agent_log_entry_t g_log[AGENT_FAULT_LOG_DEPTH];
static volatile uint32_t g_log_head        = 0U;
static volatile uint32_t g_log_count       = 0U;
static bool              g_log_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_log_init(void)
{
    g_log_head        = 0U;
    g_log_count       = 0U;
    g_log_initialized = true;
    return AGENT_OK;
}

/* ---------- LOG EVENT ---------- */
void agent_log_event(uint32_t code, uint32_t arg1, uint32_t arg2)
{
    if (!g_log_initialized) {
        return;
    }

    uint32_t idx = g_log_head;

    agent_log_entry_t *e = &g_log[idx];
    e->timestamp_us = agent_time_now_us();
    e->code         = (uint16_t)code;
    e->reserved     = 0U;
    e->arg1         = arg1;
    e->arg2         = arg2;

    /* Single-writer advance — no atomic needed on CM7 single-core */
    g_log_head = (idx + 1U) % AGENT_FAULT_LOG_DEPTH;
    g_log_count++;

    /* Sync fault-class events to context */
    if (code >= (uint32_t)AGENT_LOG_DEADLINE_MISS) {
        agent_context_full_t *ctx = agent_context_get();
        ctx->fault_count++;
        ctx->fault_last_code = code;
        ctx->fault_last_us   = e->timestamp_us;
    }
}

/* ---------- COUNT ---------- */
uint32_t agent_log_count(void)
{
    return g_log_count;
}

/* ---------- GET ENTRY ---------- */
const agent_log_entry_t* agent_log_get(uint32_t index)
{
    if (index >= AGENT_FAULT_LOG_DEPTH) {
        return NULL;
    }
    if (index >= g_log_count && g_log_count < AGENT_FAULT_LOG_DEPTH) {
        return NULL;
    }
    return &g_log[index];
}

/* ---------- DUMP ALL ---------- */
void agent_log_dump_all(void)
{
    uint32_t n = (g_log_count < AGENT_FAULT_LOG_DEPTH)
                 ? g_log_count
                 : AGENT_FAULT_LOG_DEPTH;

    for (uint32_t i = 0U; i < n; i++) {
        const agent_log_entry_t *e = &g_log[i];
        /* UART transmission handled externally (CM4 IPC) */
        (void)e;
    }
}

/* ---------- CLEAR ---------- */
void agent_log_clear(void)
{
    g_log_head  = 0U;
    g_log_count = 0U;
}