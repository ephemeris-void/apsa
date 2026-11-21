#ifndef AGENT_CERT_H
#define AGENT_CERT_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT CERT — FROZEN
 * ============================================================
 * Certification hooks: requirement pass/fail tracking.
 * DO-178C style traceability.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 50 µs max per check.
 * ============================================================ */

#define AGENT_CERT_REQUIREMENT_COUNT    16U
#define AGENT_CERT_NAME_LEN             32U

/* ---------- Status ---------- */
typedef enum {
    AGENT_CERT_STATUS_UNKNOWN       = 0,
    AGENT_CERT_STATUS_PASS          = 1,
    AGENT_CERT_STATUS_FAIL          = 2,
    AGENT_CERT_STATUS_NOT_RUN       = 3,
    AGENT_CERT_STATUS_WAIVED        = 4
} agent_cert_status_t;

/* ---------- Check callback ---------- */
typedef agent_cert_status_t (*agent_cert_check_fn_t)(void);

/* ---------- Requirement ---------- */
typedef struct {
    const char              *name;
    agent_cert_check_fn_t    check;
    agent_cert_status_t      status;
    uint32_t                 last_run_us;
    uint32_t                 last_duration_us;
    uint32_t                 run_count;
    uint32_t                 fail_count;
    uint32_t                 reserved[2];   /* ABI-safe future extension */
} agent_cert_requirement_t;

/* ---------- State ---------- */
typedef struct {
    agent_cert_requirement_t requirements[AGENT_CERT_REQUIREMENT_COUNT];
    uint32_t                 count;
    uint32_t                 passed;
    uint32_t                 failed;
    uint32_t                 total_runs;
    uint32_t                 last_run_us;
    bool                     overall_pass;
    uint32_t                 reserved[2];   /* ABI-safe future extension */
} agent_cert_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_cert_init(void);
agent_status_t                  agent_cert_reset(void);

agent_status_t                  agent_cert_register(const char           *name,
                                                    agent_cert_check_fn_t check);

agent_status_t                  agent_cert_run_all(void);
agent_status_t                  agent_cert_run_one(uint32_t idx);

bool                            agent_cert_overall_pass(void);

const agent_cert_requirement_t *agent_cert_requirement(uint32_t idx);
const agent_cert_state_t       *agent_cert_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_CERT_H */