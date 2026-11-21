#ifndef AGENT_FALLBACK_H
#define AGENT_FALLBACK_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT FALLBACK — FROZEN
 * ============================================================
 * Multi-level fallback chain for sensor data sources.
 * Each level is a function; first successful wins.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 200 µs max per execute.
 * ============================================================ */

#define AGENT_FALLBACK_MAX_LEVELS   6U

/* ---------- Levels (ordered by trust) ---------- */
typedef enum {
    AGENT_FALLBACK_LEVEL_PRIMARY    = 0,
    AGENT_FALLBACK_LEVEL_SECONDARY  = 1,
    AGENT_FALLBACK_LEVEL_TERTIARY   = 2,
    AGENT_FALLBACK_LEVEL_VIRTUAL    = 3,
    AGENT_FALLBACK_LEVEL_DEGRADED   = 4,
    AGENT_FALLBACK_LEVEL_EMERGENCY  = 5
} agent_fallback_level_t;

/* ---------- Result ---------- */
typedef enum {
    AGENT_FALLBACK_RESULT_OK           = 0,
    AGENT_FALLBACK_RESULT_DEGRADED     = 1,
    AGENT_FALLBACK_RESULT_EMERGENCY    = 2,
    AGENT_FALLBACK_RESULT_FAIL         = 3
} agent_fallback_result_t;

/* ---------- Callback ---------- */
typedef agent_status_t (*agent_fallback_fn_t)(agent_sensor_sample_t *out);

/* ---------- Entry ---------- */
typedef struct {
    agent_fallback_level_t  level;
    const char             *name;
    agent_fallback_fn_t     fn;
    uint32_t                wcet_budget_us;
    bool                    enabled;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_fallback_entry_t;

/* ---------- State ---------- */
typedef struct {
    agent_fallback_level_t  active_level;
    agent_fallback_level_t  requested_level;
    uint32_t                switch_count;
    uint32_t                fail_count;
    uint32_t                last_switch_us;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_fallback_state_t;

/* ---------- Stats ---------- */
typedef struct {
    uint32_t                total_calls;
    uint32_t                success_calls;
    uint32_t                failed_calls;
    uint32_t                level_calls[AGENT_FALLBACK_MAX_LEVELS];
    uint32_t                reserved[2];
} agent_fallback_stats_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                   agent_fallback_init(void);
agent_status_t                   agent_fallback_reset(void);

agent_status_t                   agent_fallback_register(agent_fallback_level_t level,
                                                        const char              *name,
                                                        agent_fallback_fn_t      fn,
                                                        uint32_t                 wcet_us);

agent_status_t                   agent_fallback_request(agent_fallback_level_t level);

agent_status_t                   agent_fallback_switch(agent_fallback_level_t level);

agent_status_t                   agent_fallback_execute(agent_sensor_sample_t *out,
                                                        agent_fallback_level_t *used_level);

agent_fallback_level_t           agent_fallback_active(void);

const agent_fallback_state_t    *agent_fallback_state(void);
const agent_fallback_stats_t    *agent_fallback_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_FALLBACK_H */