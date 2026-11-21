#include "agent_wcet.h"
#include "../core/agent_context.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT WCET — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per call
 * ============================================================ */

/* PERF: state + log in DTCM (zero wait state) */
static AGENT_DTCM agent_wcet_task_t  g_tasks[AGENT_WCET_TASK_MAX];
static AGENT_DTCM agent_wcet_state_t g_state;
static AGENT_DTCM agent_wcet_log_t   g_log;

/* ---------- Now in µs ---------- */
AGENT_INLINE_FAST uint32_t wcet_now_us(void)
{
    return agent_trace_cycles_to_us(agent_trace_now_cycles());
}

/* ---------- Log violation ---------- */
static void log_violation(int32_t task_id, uint32_t measured_us, uint32_t budget_us)
{
    uint32_t idx = g_log.head;

    g_log.history[idx].task_id      = (uint32_t)task_id;
    g_log.history[idx].measured_us  = measured_us;
    g_log.history[idx].budget_us    = budget_us;
    g_log.history[idx].timestamp_us = agent_time_now_us();

    g_log.head = (idx + 1U) % AGENT_WCET_HISTORY_DEPTH;
    if (g_log.count < AGENT_WCET_HISTORY_DEPTH) {
        g_log.count++;
    }
}

/* ---------- INIT ---------- */
agent_status_t agent_wcet_init(void)
{
    memset(g_tasks, 0, sizeof(g_tasks));
    memset(&g_state, 0, sizeof(g_state));    /* zero-init incl. reserved[] */
    memset(&g_log, 0, sizeof(g_log));

    g_state.last_update_us = agent_time_now_us();
    return AGENT_OK;
}

agent_status_t agent_wcet_reset(void)
{
    return agent_wcet_init();
}

/* ---------- REGISTER ---------- */
int32_t agent_wcet_register(const char *name, uint32_t budget_us)
{
    if (g_state.total_tasks >= AGENT_WCET_TASK_MAX) {
        return -1;
    }

    int32_t id = (int32_t)g_state.total_tasks;

    g_tasks[id].name          = (name != NULL) ? name : "unnamed";
    g_tasks[id].budget_us     = budget_us;
    g_tasks[id].worst_us      = 0U;
    g_tasks[id].last_us       = 0U;
    g_tasks[id].total_us      = 0U;
    g_tasks[id].call_count    = 0U;
    g_tasks[id].violate_count = 0U;
    g_tasks[id].start_cycles  = 0U;
    g_tasks[id].running       = false;
    g_tasks[id].enabled       = true;

    g_state.total_tasks++;
    return id;
}

/* ---------- BEGIN ---------- */
agent_status_t agent_wcet_begin(int32_t task_id)
{
    if (task_id < 0 || task_id >= (int32_t)g_state.total_tasks) return AGENT_ERR;
    if (!g_tasks[task_id].enabled)                              return AGENT_ERR;
    if (g_tasks[task_id].running)                               return AGENT_ERR_INVALID_STATE;

    g_tasks[task_id].start_cycles = agent_trace_now_cycles();
    g_tasks[task_id].running      = true;

    return AGENT_OK;
}

/* ---------- CHECK ---------- */
agent_status_t agent_wcet_check(int32_t task_id,
                                uint32_t measured_us,
                                agent_wcet_result_t *result_out)
{
    if (task_id < 0 || task_id >= (int32_t)g_state.total_tasks) {
        return AGENT_ERR;
    }

    agent_wcet_task_t *t = &g_tasks[task_id];

    t->last_us   = measured_us;
    t->total_us += measured_us;
    t->call_count++;

    if (measured_us > t->worst_us) {
        t->worst_us = measured_us;
    }

    if (measured_us > g_state.global_worst_us) {
        g_state.global_worst_us = measured_us;
    }

    g_state.total_calls++;
    g_state.last_update_us = agent_time_now_us();

    agent_wcet_result_t res;

    if (measured_us > t->budget_us) {
        res = AGENT_WCET_RESULT_VIOLATE;
        t->violate_count++;
        g_state.total_violates++;
        log_violation(task_id, measured_us, t->budget_us);
        agent_log_event(AGENT_LOG_WCET_VIOLATE, (uint32_t)task_id, measured_us);
    } else if (measured_us > (t->budget_us - (t->budget_us / 5U))) {
        res = AGENT_WCET_RESULT_WARN;
    } else {
        res = AGENT_WCET_RESULT_OK;
    }

    if (result_out != NULL) {
        *result_out = res;
    }

    return (res == AGENT_WCET_RESULT_VIOLATE) ? AGENT_ERR_WCET : AGENT_OK;
}

/* ---------- END ---------- */
agent_status_t agent_wcet_end(int32_t task_id, agent_wcet_result_t *result_out)
{
    if (task_id < 0 || task_id >= (int32_t)g_state.total_tasks) return AGENT_ERR;
    if (!g_tasks[task_id].running)                              return AGENT_ERR_INVALID_STATE;

    uint32_t end_cycles  = agent_trace_now_cycles();
    uint32_t dt_cycles   = end_cycles - g_tasks[task_id].start_cycles;
    uint32_t measured_us = agent_trace_cycles_to_us(dt_cycles);

    g_tasks[task_id].running = false;

    return agent_wcet_check(task_id, measured_us, result_out);
}

/* ---------- QUERIES ---------- */
const agent_wcet_task_t *agent_wcet_task(int32_t task_id)
{
    if (task_id < 0 || task_id >= (int32_t)g_state.total_tasks) {
        return NULL;
    }
    return &g_tasks[task_id];
}

const agent_wcet_state_t *agent_wcet_state(void)
{
    return &g_state;
}

const agent_wcet_log_t *agent_wcet_log(void)
{
    return &g_log;
}

/* ---------- DUMP ---------- */
void agent_wcet_dump(void)
{
    agent_log_event(AGENT_LOG_TRACE_DUMP,
                    g_state.total_calls,
                    g_state.total_violates);
    agent_log_event(AGENT_LOG_TRACE_DUMP,
                    g_state.global_worst_us,
                    g_state.total_tasks);
}