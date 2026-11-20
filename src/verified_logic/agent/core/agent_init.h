#ifndef AGENT_INIT_H
#define AGENT_INIT_H

#include "agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT SYSTEM INIT — FROZEN
 * ============================================================
 * Full initialization sequence.
 * Called ONCE from agent_main_init() at boot.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * ============================================================ */

agent_status_t agent_system_init(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_INIT_H */