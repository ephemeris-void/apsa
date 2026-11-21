#ifndef AGENT_POLICY_WATCHDOG_H
#define AGENT_POLICY_WATCHDOG_H

#include "../core/agent_types.h"
#include "../policy/agent_policy_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY WATCHDOG — FROZEN
 * ============================================================
 * Per-policy liveness + latency + failure tracking.
 * Detects slow/failed policies; disables them after N failures.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5 µs max per kick.
 * ============================================================ */

#define AGENT_POLICY_WDT_TIMEOUT_US        50000U
#define AGENT_POLICY_WDT_MAX_LATENCY_US    5000U
#define AGENT_POLICY_WDT_MAX_FAILS         3U

/* ---------- Result ---------- */
typedef enum {
    AGENT_POLICY_WDT_OK        = 0,
    AGENT_POLICY_WDT_WARN      = 1,
    AGENT_POLICY_WDT_TIMEOUT   = 2,
    AGENT_POLICY_WDT_FAILED    = 3
} agent_policy_wdt_result_t;

/* ---------- Entry ---------- */
typedef struct {
    agent_policy_id_t         policy_id;
    uint32_t                  last_call_us;
    uint32_t                  last_latency_us;
    uint32_t                  worst_latency_us;
    uint32_t                  call_count;
    uint32_t                  timeout_count;
    uint32_t                  fail_count;
    agent_policy_wdt_result_t result;
} agent_policy_wdt_entry_t;

/* ---------- State ---------- */
typedef struct {
    agent_policy_wdt_entry_t entries[AGENT_POLICY_ID_COUNT];
    uint32_t                 total_calls;
    uint32_t                 total_timeouts;
    uint32_t                 total_fails;
    uint32_t                 last_update_us;
} agent_policy_wdt_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                        agent_policy_watchdog_init(void);
agent_status_t                        agent_policy_watchdog_reset(void);

agent_status_t                        agent_policy_watchdog_kick(agent_policy_id_t id,
                                                               uint32_t          latency_us,
                                                               agent_status_t    status);

void                                  agent_policy_watchdog_update(void);

bool                                  agent_policy_watchdog_is_healthy(agent_policy_id_t id);

const agent_policy_wdt_entry_t       *agent_policy_watchdog_entry(agent_policy_id_t id);
const agent_policy_wdt_state_t       *agent_policy_watchdog_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_WATCHDOG_H */