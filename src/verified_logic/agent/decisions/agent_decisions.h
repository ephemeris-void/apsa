#ifndef AGENT_DECISIONS_H
#define AGENT_DECISIONS_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT DECISIONS — FROZEN
 * ============================================================
 * Decision engine: perceives → decides → outputs decision.
 * WCET: 200 µs max.
 * Deterministic. No randomness. No dynamic allocation.
 * ============================================================ */

/* ---------- Decision API (FROZEN) ---------- */
agent_status_t  agent_decide(agent_decision_t *out);

/* ---------- Perception (FROZEN) ---------- */
void            agent_decide_perceive(void);

/* ---------- Reasoning (FROZEN) ---------- */
agent_state_t   agent_decide_next_state(void);
uint32_t        agent_decide_reason(void);

/* ---------- Priority (FROZEN) ---------- */
agent_priority_t agent_decide_priority(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_DECISIONS_H */