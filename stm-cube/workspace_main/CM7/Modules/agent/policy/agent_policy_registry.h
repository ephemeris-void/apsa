#ifndef AGENT_POLICY_REGISTRY_H
#define AGENT_POLICY_REGISTRY_H

#include "agent_policy_if.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY REGISTRY — FROZEN
 * ============================================================
 * Runtime-registerable policy table.
 * New policy = register() at init, no code edit elsewhere.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per register/lookup.
 * ============================================================ */

agent_status_t              agent_policy_registry_reset(void);
agent_status_t              agent_policy_registry_register(const agent_policy_desc_t *desc);
agent_status_t              agent_policy_registry_unregister(agent_policy_id_t id);
uint32_t                    agent_policy_registry_count(void);
const agent_policy_desc_t  *agent_policy_registry_at(uint32_t index);
const agent_policy_desc_t  *agent_policy_registry_find(agent_policy_id_t id);
agent_status_t              agent_policy_registry_sort_by_priority(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_REGISTRY_H */