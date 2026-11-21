#ifndef AGENT_DATALOG_RING_H
#define AGENT_DATALOG_RING_H

#include "agent_datalog.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT DATALOG RING — FROZEN
 * ============================================================
 * Ring buffer capacity/usage queries.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 1 µs max per call.
 * ============================================================ */

agent_status_t  agent_datalog_ring_init(void);
agent_status_t  agent_datalog_ring_reset(void);
uint32_t        agent_datalog_ring_capacity(void);
uint32_t        agent_datalog_ring_usage(void);
bool            agent_datalog_ring_is_full(void);
bool            agent_datalog_ring_is_empty(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_DATALOG_RING_H */