#ifndef AGENT_TRACE_H
#define AGENT_TRACE_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT TRACE — FROZEN
 * ============================================================
 * Cycle-level instrumentation via DWT->CYCCNT.
 * Every critical function is measured.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5 µs max per trace call.
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  agent_trace_init(void);

/* ---------- Loop Trace ---------- */
void            agent_trace_loop_begin(void);
void            agent_trace_loop_end(void);

/* ---------- State Trace ---------- */
void            agent_trace_state_enter(agent_state_t state);
void            agent_trace_state_begin(agent_state_t state);
void            agent_trace_state_end(agent_state_t state);

/* ---------- Decision Trace ---------- */
void            agent_trace_decision_begin(void);
void            agent_trace_decision_end(void);

/* ---------- Action Trace ---------- */
void            agent_trace_action_begin(uint32_t action_id);
void            agent_trace_action_end(uint32_t action_id);

/* ---------- Safety Trace ---------- */
void            agent_trace_safety_begin(void);
void            agent_trace_safety_end(void);

/* ---------- Transition Trace ---------- */
void            agent_trace_transition(agent_state_t from,
                                        agent_state_t to,
                                        uint32_t      reason);

/* ---------- AI Trace ---------- */
void            agent_trace_ai_begin(void);
void            agent_trace_ai_end(void);

/* ---------- Cycle Counter API ---------- */
uint32_t        agent_trace_now_cycles(void);
uint32_t        agent_trace_cycles_to_us(uint32_t cycles);
void            agent_trace_dump_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_TRACE_H */