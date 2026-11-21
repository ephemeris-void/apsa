#ifndef AGENT_BT_H
#define AGENT_BT_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT BEHAVIOR TREE — FROZEN
 * ============================================================
 * Deterministic behavior tree engine.
 * Node types: sequence, selector, parallel, inverter, repeat, leaf.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 500 µs max per tick.
 * ============================================================ */

#define AGENT_BT_MAX_NODES          32U
#define AGENT_BT_MAX_CHILDREN       6U
#define AGENT_BT_TICK_TIMEOUT_US    500U

/* ---------- Status ---------- */
typedef enum {
    AGENT_BT_STATUS_INVALID     = 0,
    AGENT_BT_STATUS_SUCCESS     = 1,
    AGENT_BT_STATUS_FAILURE     = 2,
    AGENT_BT_STATUS_RUNNING     = 3
} agent_bt_status_t;

/* ---------- Node types ---------- */
typedef enum {
    AGENT_BT_NODE_SEQUENCE      = 0,
    AGENT_BT_NODE_SELECTOR      = 1,
    AGENT_BT_NODE_PARALLEL      = 2,
    AGENT_BT_NODE_ACTION        = 3,
    AGENT_BT_NODE_CONDITION     = 4,
    AGENT_BT_NODE_INVERTER      = 5,
    AGENT_BT_NODE_REPEAT        = 6
} agent_bt_node_type_t;

/* ---------- Tick callback ---------- */
typedef agent_bt_status_t (*agent_bt_tick_fn_t)(void *ctx);

/* ---------- Node ---------- */
typedef struct {
    uint32_t                id;
    agent_bt_node_type_t    type;
    const char             *name;
    agent_bt_tick_fn_t      tick;
    void                   *ctx;
    int32_t                 children[AGENT_BT_MAX_CHILDREN];
    uint32_t                child_count;
    agent_bt_status_t       last_status;
    uint32_t                tick_count;
    uint32_t                success_count;
    uint32_t                failure_count;
} agent_bt_node_t;

/* ---------- Tree ---------- */
typedef struct {
    agent_bt_node_t         nodes[AGENT_BT_MAX_NODES];
    uint32_t                node_count;
    int32_t                 root_id;
    int32_t                 current_id;
    uint32_t                total_ticks;
    uint32_t                last_tick_us;
    agent_bt_status_t       last_status;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_bt_tree_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_bt_init(void);
agent_status_t              agent_bt_reset(void);

int32_t                     agent_bt_add_node(agent_bt_node_type_t type,
                                              const char          *name,
                                              agent_bt_tick_fn_t   tick,
                                              void                *ctx);

agent_status_t              agent_bt_add_child(int32_t parent_id, int32_t child_id);

agent_status_t              agent_bt_set_root(int32_t node_id);

agent_bt_status_t           agent_bt_tick(void);

agent_bt_status_t           agent_bt_node_status(int32_t node_id);
int32_t                     agent_bt_current(void);

const agent_bt_tree_t      *agent_bt_tree(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_BT_H */