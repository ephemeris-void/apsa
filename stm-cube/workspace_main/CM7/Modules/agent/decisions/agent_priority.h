#ifndef AGENT_PRIORITY_H
#define AGENT_PRIORITY_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT PRIORITY ARBITRATION — FROZEN
 * ============================================================
 * Determines execution priority when multiple decisions compete.
 * Deterministic. Priority = highest-severity wins.
 * WCET: 50 µs max.
 * ============================================================ */

/* ---------- Priority API (FROZEN) ---------- */
agent_priority_t    agent_priority_evaluate(void);
agent_priority_t    agent_priority_from_state(agent_state_t state);
agent_priority_t    agent_priority_from_health(agent_health_t health);
agent_priority_t    agent_priority_max(agent_priority_t a,
                                       agent_priority_t b);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_PRIORITY_H */