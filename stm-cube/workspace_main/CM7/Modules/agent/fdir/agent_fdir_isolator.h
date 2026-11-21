#ifndef AGENT_FDIR_ISOLATOR_H
#define AGENT_FDIR_ISOLATOR_H

#include "agent_fdir.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT FDIR ISOLATOR — FROZEN
 * ============================================================
 * Fault isolation: marks sensor failed, triggers driver fail.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per run.
 * ============================================================ */

agent_status_t  agent_fdir_isolator_init(void);
agent_status_t  agent_fdir_isolator_reset(void);

agent_status_t  agent_fdir_isolator_run(agent_fdir_channel_t *ch);

agent_status_t  agent_fdir_isolator_apply(agent_fdir_sensor_t sensor,
                                          agent_fdir_fault_t  fault);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_FDIR_ISOLATOR_H */