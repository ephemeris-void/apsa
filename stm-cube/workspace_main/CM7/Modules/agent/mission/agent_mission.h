#ifndef AGENT_MISSION_H
#define AGENT_MISSION_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MISSION — FROZEN
 * ============================================================
 * Waypoint-based mission execution with state machine.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 100 µs max per update.
 * ============================================================ */

#define AGENT_MISSION_MAX_WAYPOINTS     32U
#define AGENT_MISSION_MAX_NAME_LEN      32U
#define AGENT_MISSION_TIMEOUT_US        600000000U   /* 10 min */

/* ---------- Mission State ---------- */
typedef enum {
    AGENT_MISSION_IDLE          = 0,
    AGENT_MISSION_ARMED         = 1,
    AGENT_MISSION_TAKEOFF       = 2,
    AGENT_MISSION_ENROUTE       = 3,
    AGENT_MISSION_LOITER        = 4,
    AGENT_MISSION_RTL           = 5,
    AGENT_MISSION_LANDING       = 6,
    AGENT_MISSION_COMPLETE      = 7,
    AGENT_MISSION_ABORTED       = 8,
    AGENT_MISSION_FAILED        = 9
} agent_mission_state_t;

/* ---------- Waypoint Type ---------- */
typedef enum {
    AGENT_MISSION_WP_TYPE_NONE      = 0,
    AGENT_MISSION_WP_TYPE_TAKEOFF   = 1,
    AGENT_MISSION_WP_TYPE_WAYPOINT  = 2,
    AGENT_MISSION_WP_TYPE_LOITER    = 3,
    AGENT_MISSION_WP_TYPE_LAND      = 4,
    AGENT_MISSION_WP_TYPE_RTL       = 5
} agent_mission_wp_type_t;

/* ---------- Waypoint ---------- */
typedef struct {
    float                    pos[3];
    float                    speed;
    float                    hold_time_s;
    agent_mission_wp_type_t  type;
    uint32_t                 reserved[2];   /* ABI-safe future extension */
} agent_mission_waypoint_t;

/* ---------- Context ---------- */
typedef struct {
    agent_mission_state_t    state;
    agent_mission_state_t    previous;
    uint32_t                 current_wp;
    uint32_t                 wp_count;
    uint32_t                 start_us;
    uint32_t                 elapsed_us;
    uint32_t                 state_entry_us;
    uint32_t                 loops_completed;
    bool                     active;
    uint32_t                 reserved[2];   /* ABI-safe future extension */
} agent_mission_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_mission_init(void);
agent_status_t                  agent_mission_reset(void);
agent_status_t                  agent_mission_update(void);

agent_status_t                  agent_mission_load(const agent_mission_waypoint_t *wps,
                                                   uint32_t                        count);

agent_status_t                  agent_mission_start(void);
agent_status_t                  agent_mission_abort(void);
agent_status_t                  agent_mission_hold(void);
agent_status_t                  agent_mission_resume(void);

agent_status_t                  agent_mission_advance(void);

agent_mission_state_t           agent_mission_state(void);
uint32_t                        agent_mission_current_wp(void);
const agent_mission_waypoint_t *agent_mission_waypoint(uint32_t idx);

const agent_mission_ctx_t      *agent_mission_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MISSION_H */