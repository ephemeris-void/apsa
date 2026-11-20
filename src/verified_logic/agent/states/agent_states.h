#ifndef AGENT_STATES_H
#define AGENT_STATES_H

#include "../core/agent_types.h"
#include "../core/agent_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT STATES — FROZEN
 * ============================================================
 * 8 states. Each state = one handler function.
 * No state handler may block. No dynamic allocation.
 * WCET: 200 µs per state handler max.
 * ============================================================ */

/* ---------- State Lifecycle (FROZEN) ---------- */
void agent_state_set(agent_state_t new_state);
void agent_state_enter(agent_state_t state);
void agent_state_exit(agent_state_t state);
void agent_state_run(agent_state_t state);

/* ---------- Individual State Handlers (FROZEN) ---------- */
void agent_state_boot(void);
void agent_state_monitor(void);
void agent_state_degraded(void);
void agent_state_virtualize(void);
void agent_state_fusion(void);
void agent_state_recovery(void);
void agent_state_emergency(void);
void agent_state_landing(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_STATES_H */