#ifndef AGENT_POLICY_H
#define AGENT_POLICY_H

#include "agent_policy_if.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY — FROZEN
 * ============================================================
 * Candidate arbitration and application.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 50 µs max per arbitrate.
 * ============================================================ */

#define AGENT_POLICY_REGISTRY_MAX   16U

/* ---------- Candidate ---------- */
typedef struct {
    agent_policy_id_t    id;
    agent_priority_t     priority;
    agent_state_t        target_state;
    agent_reason_t       reason;
    uint32_t             confidence_milli;
    uint32_t             timestamp_us;
    uint32_t             wcet_us;
    uint32_t             reserved[2];   /* ABI-safe future extension */
} agent_policy_candidate_t;

/* ---------- Arbitration ---------- */
typedef struct {
    agent_policy_candidate_t candidates[AGENT_POLICY_REGISTRY_MAX];
    uint32_t                 count;
    agent_policy_id_t        winner_id;
    uint32_t                 timestamp_us;
} agent_policy_arbitration_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t agent_policy_collect(agent_policy_candidate_t *out,
                                    uint32_t                  max,
                                    uint32_t                 *count_out);

agent_status_t agent_policy_arbitrate(const agent_policy_candidate_t *in,
                                      uint32_t                        count,
                                      agent_policy_candidate_t       *winner_out);

agent_status_t agent_policy_apply(const agent_policy_candidate_t *winner);

agent_status_t agent_policy_safety_filter(agent_policy_candidate_t *candidate);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_H */