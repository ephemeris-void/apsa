#ifndef AGENT_POLICY_IF_H
#define AGENT_POLICY_IF_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY INTERFACE — FROZEN
 * ============================================================
 * Pluggable policy evaluation: each policy produces a candidate.
 * Registry + arbiter select the winner deterministically.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * ============================================================ */

/* ---------- Eval Result ---------- */
typedef enum {
    AGENT_POLICY_OK        = 0,
    AGENT_POLICY_DEGRADED  = 1,
    AGENT_POLICY_FAIL      = 2,
    AGENT_POLICY_DIVERGE   = 3,
    AGENT_POLICY_STALE     = 4
} agent_policy_eval_t;

/* ---------- Policy IDs ---------- */
typedef enum {
    AGENT_POLICY_ID_NONE      = 0x00,
    AGENT_POLICY_ID_NOMINAL   = 0x01,
    AGENT_POLICY_ID_DEGRADED  = 0x02,
    AGENT_POLICY_ID_VIRTUAL   = 0x03,
    AGENT_POLICY_ID_FUSION    = 0x04,
    AGENT_POLICY_ID_RECOVERY  = 0x05,
    AGENT_POLICY_ID_EMERGENCY = 0x06,
    AGENT_POLICY_ID_LANDING   = 0x07,
    AGENT_POLICY_ID_FDIR      = 0x08,
    AGENT_POLICY_ID_MLP       = 0x09,
    AGENT_POLICY_ID_RL        = 0x0A,
    AGENT_POLICY_ID_TF        = 0x0B,
    AGENT_POLICY_ID_COUNT     = 12
} agent_policy_id_t;

/* ---------- Policy Output ---------- */
typedef struct {
    agent_policy_eval_t  result;
    agent_priority_t     priority;
    agent_state_t        suggested_state;
    uint32_t             confidence_milli;
    uint32_t             reason_code;
    uint32_t             timestamp_us;
    uint32_t             wcet_us;
    uint32_t             reserved[2];   /* ABI-safe future extension */
} agent_policy_output_t;

/* ---------- Policy Descriptor ---------- */
typedef struct {
    agent_policy_id_t    id;
    const char          *name;

    agent_status_t      (*init)(void);
    agent_status_t      (*eval)(agent_policy_output_t *out);
    agent_status_t      (*reset)(void);

    uint32_t             wcet_budget_us;
    uint32_t             priority_level;
} agent_policy_desc_t;

/* ---------- Registry API (FROZEN) ---------- */
agent_status_t agent_policy_init(agent_policy_id_t id);
agent_status_t agent_policy_eval(agent_policy_id_t id, agent_policy_output_t *out);
agent_status_t agent_policy_reset(agent_policy_id_t id);

const agent_policy_desc_t *agent_policy_get_desc(agent_policy_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_IF_H */