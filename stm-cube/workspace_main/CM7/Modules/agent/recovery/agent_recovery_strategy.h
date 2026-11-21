#ifndef AGENT_RECOVERY_STRATEGY_H
#define AGENT_RECOVERY_STRATEGY_H

#include "agent_recovery.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT RECOVERY STRATEGY — FROZEN
 * ============================================================
 * Concrete recovery actions (restart, reset, reconfigure).
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 50 µs max per apply.
 * ============================================================ */

agent_status_t              agent_recovery_strategy_init(void);
agent_status_t              agent_recovery_strategy_apply(agent_recovery_strategy_t strategy);
agent_recovery_strategy_t   agent_recovery_strategy_status(void);
void                        agent_recovery_strategy_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_RECOVERY_STRATEGY_H */