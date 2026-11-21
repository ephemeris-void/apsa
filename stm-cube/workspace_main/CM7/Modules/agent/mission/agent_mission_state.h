#ifndef AGENT_MISSION_STATE_H
#define AGENT_MISSION_STATE_H

#include "agent_mission.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MISSION STATE — FROZEN
 * ============================================================
 * Per-state entry + step handlers.
 * Deterministic per-state behavior.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 50 µs max per step.
 * ============================================================ */

agent_status_t  agent_mission_state_init(void);
agent_status_t  agent_mission_state_enter(agent_mission_state_t state);
agent_status_t  agent_mission_state_step(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MISSION_STATE_H */