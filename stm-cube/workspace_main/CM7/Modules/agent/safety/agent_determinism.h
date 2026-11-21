#ifndef AGENT_DETERMINISM_H
#define AGENT_DETERMINISM_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT DETERMINISM — FROZEN
 * ============================================================
 * Same input → same output verification via state hash history.
 * Detects non-deterministic behavior in agent loop.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5 µs max per check.
 * ============================================================ */

#define AGENT_DET_HISTORY_DEPTH     16U
#define AGENT_DET_HASH_SIZE         256U

/* ---------- Result ---------- */
typedef enum {
    AGENT_DET_RESULT_OK         = 0,
    AGENT_DET_RESULT_MISMATCH   = 1,
    AGENT_DET_RESULT_FAIL       = 2,
    AGENT_DET_RESULT_DISABLED   = 3
} agent_det_result_t;

/* ---------- Sample ---------- */
typedef struct {
    uint32_t    state;
    uint32_t    priority;
    uint32_t    action_id;
    uint32_t    iteration;
    uint32_t    timestamp_us;
    uint32_t    hash;
} agent_det_sample_t;

/* ---------- State ---------- */
typedef struct {
    agent_det_sample_t  history[AGENT_DET_HISTORY_DEPTH];
    uint32_t            head;
    uint32_t            count;
    uint32_t            check_count;
    uint32_t            mismatch_count;
    uint32_t            last_check_us;
    bool                enabled;
    uint32_t            reserved[2];   /* ABI-safe future extension */
} agent_det_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_determinism_init(void);
agent_status_t              agent_determinism_reset(void);
agent_status_t              agent_determinism_enable(bool en);

uint32_t                    agent_determinism_hash(uint32_t state,
                                                   uint32_t priority,
                                                   uint32_t action_id,
                                                   uint32_t iteration);

agent_status_t              agent_determinism_record(void);

agent_det_result_t          agent_determinism_check(uint32_t iteration);

void                        agent_determinism_clear(void);

const agent_det_state_t    *agent_determinism_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_DETERMINISM_H */