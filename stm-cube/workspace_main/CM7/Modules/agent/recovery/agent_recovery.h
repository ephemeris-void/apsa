#ifndef AGENT_RECOVERY_H
#define AGENT_RECOVERY_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT RECOVERY — FROZEN
 * ============================================================
 * Fault recovery state machine with bounded retries.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 100 µs max per update.
 * ============================================================ */

#define AGENT_RECOVERY_MAX_ATTEMPTS     3U
#define AGENT_RECOVERY_WINDOW_US        30000000U   /* 30 s */
#define AGENT_RECOVERY_MIN_DWELL_US     500U

/* ---------- Recovery State ---------- */
typedef enum {
    AGENT_RECOVERY_IDLE         = 0,
    AGENT_RECOVERY_ATTEMPTING   = 1,
    AGENT_RECOVERY_VERIFYING    = 2,
    AGENT_RECOVERY_SUCCEEDED    = 3,
    AGENT_RECOVERY_FAILED       = 4,
    AGENT_RECOVERY_EXHAUSTED    = 5
} agent_recovery_state_t;

/* ---------- Recovery Strategy ---------- */
typedef enum {
    AGENT_RECOVERY_STRATEGY_NONE        = 0,
    AGENT_RECOVERY_STRATEGY_RESTART     = 1,
    AGENT_RECOVERY_STRATEGY_RESET       = 2,
    AGENT_RECOVERY_STRATEGY_RECONFIG    = 3,
    AGENT_RECOVERY_STRATEGY_ISOLATE     = 4,
    AGENT_RECOVERY_STRATEGY_DEGRADE     = 5
} agent_recovery_strategy_t;

/* ---------- Recovery Context ---------- */
typedef struct {
    agent_recovery_state_t      state;
    agent_recovery_strategy_t   strategy;
    uint32_t                    attempts;
    uint32_t                    max_attempts;
    uint32_t                    start_us;
    uint32_t                    last_attempt_us;
    uint32_t                    success_count;
    uint32_t                    fail_count;
    uint32_t                    window_start_us;
} agent_recovery_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_recovery_init(void);
agent_status_t              agent_recovery_reset(void);
agent_status_t              agent_recovery_update(void);

agent_status_t              agent_recovery_start(agent_recovery_strategy_t strategy);
agent_status_t              agent_recovery_abort(void);

agent_recovery_state_t      agent_recovery_state(void);
agent_recovery_strategy_t   agent_recovery_strategy(void);

const agent_recovery_ctx_t *agent_recovery_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_RECOVERY_H */