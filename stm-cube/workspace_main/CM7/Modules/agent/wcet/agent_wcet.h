#ifndef AGENT_WCET_H
#define AGENT_WCET_H

#include "../core/agent_types.h"
#include "../core/agent_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT WCET — FROZEN
 * ============================================================
 * Per-task WCET budget enforcement via DWT cycle counter.
 * Records violations to a ring buffer for post-mortem.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5 µs max per call.
 * ============================================================ */

#define AGENT_WCET_TASK_MAX         16U
#define AGENT_WCET_HISTORY_DEPTH    16U

/* ---------- Result ---------- */
typedef enum {
    AGENT_WCET_RESULT_OK        = 0,
    AGENT_WCET_RESULT_WARN      = 1,
    AGENT_WCET_RESULT_VIOLATE   = 2,
    AGENT_WCET_RESULT_FAIL      = 3
} agent_wcet_result_t;

/* ---------- Task ---------- */
typedef struct {
    const char *name;
    uint32_t    budget_us;
    uint32_t    worst_us;
    uint32_t    last_us;
    uint32_t    total_us;
    uint32_t    call_count;
    uint32_t    violate_count;
    uint32_t    start_cycles;
    bool        running;
    bool        enabled;
} agent_wcet_task_t;

/* ---------- State ---------- */
typedef struct {
    uint32_t    total_tasks;
    uint32_t    total_calls;
    uint32_t    total_violates;
    uint32_t    global_worst_us;
    uint32_t    last_update_us;
    uint32_t    reserved[2];   /* ABI-safe future extension */
} agent_wcet_state_t;

/* ---------- Violation ---------- */
typedef struct {
    uint32_t    task_id;
    uint32_t    measured_us;
    uint32_t    budget_us;
    uint32_t    timestamp_us;
} agent_wcet_violation_t;

/* ---------- Log ---------- */
typedef struct {
    agent_wcet_violation_t history[AGENT_WCET_HISTORY_DEPTH];
    uint32_t               head;
    uint32_t               count;
} agent_wcet_log_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                agent_wcet_init(void);
agent_status_t                agent_wcet_reset(void);

int32_t                       agent_wcet_register(const char *name,
                                                  uint32_t    budget_us);

agent_status_t                agent_wcet_begin(int32_t task_id);

agent_status_t                agent_wcet_end(int32_t task_id,
                                             agent_wcet_result_t *result_out);

agent_status_t                agent_wcet_check(int32_t task_id,
                                               uint32_t measured_us,
                                               agent_wcet_result_t *result_out);

const agent_wcet_task_t      *agent_wcet_task(int32_t task_id);
const agent_wcet_state_t     *agent_wcet_state(void);
const agent_wcet_log_t       *agent_wcet_log(void);

void                          agent_wcet_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_WCET_H */