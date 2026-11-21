#ifndef AGENT_GOAL_H
#define AGENT_GOAL_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT GOAL — FROZEN
 * ============================================================
 * Goal-based planning layer: set, plan, execute, monitor.
 * Multiple concurrent goals with priority arbitration.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 100 µs max per update.
 * ============================================================ */

#define AGENT_GOAL_MAX_ACTIVE       4U
#define AGENT_GOAL_MAX_STEPS        16U
#define AGENT_GOAL_HISTORY_DEPTH    8U

/* ---------- Goal IDs ---------- */
typedef enum {
    AGENT_GOAL_ID_NONE          = 0x00,
    AGENT_GOAL_ID_HOVER         = 0x01,
    AGENT_GOAL_ID_WAYPOINT      = 0x02,
    AGENT_GOAL_ID_LAND          = 0x03,
    AGENT_GOAL_ID_RTL           = 0x04,
    AGENT_GOAL_ID_LOITER        = 0x05,
    AGENT_GOAL_ID_AVOID         = 0x06,
    AGENT_GOAL_ID_RECOVER       = 0x07,
    AGENT_GOAL_ID_EMERGENCY     = 0x08,
    AGENT_GOAL_ID_COUNT         = 9
} agent_goal_id_t;

/* ---------- Goal State ---------- */
typedef enum {
    AGENT_GOAL_STATE_IDLE       = 0,
    AGENT_GOAL_STATE_PLANNING   = 1,
    AGENT_GOAL_STATE_EXECUTING  = 2,
    AGENT_GOAL_STATE_MONITORING = 3,
    AGENT_GOAL_STATE_DONE       = 4,
    AGENT_GOAL_STATE_FAILED     = 5,
    AGENT_GOAL_STATE_ABORTED    = 6
} agent_goal_state_t;

/* ---------- Goal Priority ---------- */
typedef enum {
    AGENT_GOAL_PRIO_LOW         = 0,
    AGENT_GOAL_PRIO_NORMAL      = 1,
    AGENT_GOAL_PRIO_HIGH        = 2,
    AGENT_GOAL_PRIO_CRITICAL    = 3
} agent_goal_prio_t;

/* ---------- Goal ---------- */
typedef struct {
    agent_goal_id_t     id;
    agent_goal_state_t  state;
    agent_goal_prio_t   priority;
    uint32_t            step_count;
    uint32_t            start_us;
    uint32_t            deadline_us;
    float               target[3];
    float               tolerance;
    uint32_t            progress_milli;
    bool                active;
    uint32_t            reserved[2];   /* ABI-safe future extension */
} agent_goal_t;

/* ---------- Context ---------- */
typedef struct {
    agent_goal_t        active[AGENT_GOAL_MAX_ACTIVE];
    uint32_t            active_count;
    agent_goal_id_t     current;
    agent_goal_id_t     previous;
    uint32_t            completed;
    uint32_t            failed;
    uint32_t            aborted;
    uint32_t            last_update_us;
    uint32_t            reserved[2];
} agent_goal_state_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_goal_init(void);
agent_status_t                  agent_goal_reset(void);
agent_status_t                  agent_goal_update(void);

agent_status_t                  agent_goal_set(agent_goal_id_t     id,
                                               agent_goal_prio_t   prio,
                                               const float         target[3],
                                               float               tolerance,
                                               uint32_t            deadline_us);

agent_status_t                  agent_goal_abort(agent_goal_id_t id);
agent_status_t                  agent_goal_complete(agent_goal_id_t id);

agent_goal_id_t                 agent_goal_current(void);
agent_goal_state_t              agent_goal_state(agent_goal_id_t id);

const agent_goal_t             *agent_goal_get(agent_goal_id_t id);
const agent_goal_state_ctx_t   *agent_goal_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_GOAL_H */