#ifndef AGENT_POLICY_FALLBACK_H
#define AGENT_POLICY_FALLBACK_H

#include "../core/agent_types.h"
#include "../policy/agent_policy_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY FALLBACK — FROZEN
 * ============================================================
 * Cascading fallback chain for policy selection:
 *   primary → secondary → rule-based → safe default.
 * Used when registered policies fail or are unhealthy.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per call.
 * ============================================================ */

#define AGENT_POLICY_FB_MAX_LEVELS     4U

/* ---------- Levels ---------- */
typedef enum {
    AGENT_POLICY_FB_LEVEL_PRIMARY   = 0,
    AGENT_POLICY_FB_LEVEL_SECONDARY = 1,
    AGENT_POLICY_FB_LEVEL_RULE      = 2,
    AGENT_POLICY_FB_LEVEL_SAFE      = 3
} agent_policy_fb_level_t;

/* ---------- Entry ---------- */
typedef struct {
    agent_policy_fb_level_t level;
    agent_policy_id_t       policy_id;
    uint32_t                switch_count;
    uint32_t                fail_count;
    uint32_t                last_switch_us;
} agent_policy_fb_entry_t;

/* ---------- State ---------- */
typedef struct {
    agent_policy_fb_entry_t entries[AGENT_POLICY_FB_MAX_LEVELS];
    agent_policy_fb_level_t active_level;
    uint32_t                total_switches;
    uint32_t                total_fails;
    uint32_t                last_update_us;
} agent_policy_fb_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                       agent_policy_fallback_init(void);
agent_status_t                       agent_policy_fallback_reset(void);

agent_status_t                       agent_policy_fallback_register(agent_policy_fb_level_t level,
                                                                  agent_policy_id_t       policy_id);

agent_policy_fb_level_t              agent_policy_fallback_next(agent_policy_fb_level_t current);

agent_status_t                       agent_policy_fallback_switch(agent_policy_fb_level_t level);

agent_policy_fb_level_t              agent_policy_fallback_active(void);

const agent_policy_fb_entry_t       *agent_policy_fallback_entry(agent_policy_fb_level_t level);
const agent_policy_fb_state_t       *agent_policy_fallback_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_FALLBACK_H */