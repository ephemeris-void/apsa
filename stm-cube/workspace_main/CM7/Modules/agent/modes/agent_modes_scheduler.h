#ifndef AGENT_MODES_SCHEDULER_H
#define AGENT_MODES_SCHEDULER_H

#include "agent_modes.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MODES SCHEDULER — FROZEN
 * ============================================================
 * Anti-chatter: enforces minimum dwell time per mode.
 * EMERGENCY always allowed (bypasses dwell).
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per call.
 * ============================================================ */

agent_status_t  agent_modes_scheduler_init(void);
agent_status_t  agent_modes_scheduler_reset(void);

bool            agent_modes_scheduler_allow_switch(agent_mode_id_t from,
                                                   agent_mode_id_t to,
                                                   uint32_t        dwell_us);

uint32_t        agent_modes_scheduler_next_period_us(agent_mode_id_t id);

uint32_t        agent_modes_scheduler_slot_count(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MODES_SCHEDULER_H */