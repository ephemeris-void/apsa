#ifndef AGENT_MODES_ARBITER_H
#define AGENT_MODES_ARBITER_H

#include "agent_modes.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MODES ARBITER — FROZEN
 * ============================================================
 * Selects desired mode based on system state.
 * Deterministic — no randomness.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per select.
 * ============================================================ */

agent_status_t  agent_modes_arbiter_init(void);
agent_status_t  agent_modes_arbiter_reset(void);

agent_mode_id_t agent_modes_arbiter_select(void);

agent_status_t  agent_modes_arbiter_force(agent_mode_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MODES_ARBITER_H */