#ifndef AGENT_MAIN_HOOK_H
#define AGENT_MAIN_HOOK_H

#include "agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MAIN HOOK — FROZEN
 * ============================================================
 * Integration interface with main.c / Zephyr thread.
 * This is the ONLY file main.c should include from the agent.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * ============================================================ */

/* ---------- Called once at boot ---------- */
agent_status_t  agent_main_init(void);

/* ---------- Called every iteration (1 kHz target) ---------- */
agent_status_t  agent_main_step(void);

/* ---------- Called on shutdown ---------- */
agent_status_t  agent_main_shutdown(void);

/* ============================================================
 * ZEPHYR HOOK (uncomment when Zephyr integrated)
 * ============================================================ */
/* void            agent_zephyr_start(void); */

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MAIN_HOOK_H */