#ifndef AGENT_TRANSITIONS_H
#define AGENT_TRANSITIONS_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT TRANSITIONS — FROZEN
 * ============================================================
 * State transition rules and guards.
 * Each transition is deterministic. No dynamic decisions.
 * Allowed transitions are predefined. Others are BLOCKED.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 30 µs max.
 * ============================================================ */

/* ---------- Transition API (FROZEN) ---------- */
agent_status_t  agent_transition_to(agent_state_t new_state,
                                    uint32_t     reason_code);

/* ---------- Transition Guard (FROZEN) ---------- */
bool            agent_transition_is_allowed(agent_state_t from,
                                            agent_state_t to);

/* ---------- Direct Transition (bypasses guard — SAFETY use only) ---------- */
agent_status_t  agent_transition_force(agent_state_t new_state,
                                       uint32_t     reason_code);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_TRANSITIONS_H */