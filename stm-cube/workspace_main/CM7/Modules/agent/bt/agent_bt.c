#include "agent_bt.h"
#include "agent_bt_nodes.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT BEHAVIOR TREE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 500 µs per tick
 * ============================================================ */

/* PERF: tree in DTCM (zero wait state) */
static AGENT_DTCM agent_bt_tree_t g_bt;
static bool                       g_bt_initialized = false;

static agent_bt_status_t tick_node(int32_t node_id, uint32_t depth);

/* ---------- INIT ---------- */
agent_status_t agent_bt_init(void)
{
    memset(&g_bt, 0, sizeof(g_bt));    /* zero-init incl. reserved[] */

    g_bt.node_count   = 0U;
    g_bt.root_id      = -1;
    g_bt.current_id   = -1;
    g_bt.total_ticks  = 0U;
    g_bt.last_tick_us = 0U;
    g_bt.last_status  = AGENT_BT_STATUS_INVALID;

    agent_bt_nodes_init();

    g_bt_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_bt_reset(void)
{
    g_bt_initialized = false;
    return agent_bt_init();
}

/* ---------- ADD NODE ---------- */
int32_t agent_bt_add_node(agent_bt_node_type_t type,
                          const char          *name,
                          agent_bt_tick_fn_t   tick,
                          void                *ctx)
{
    if (!g_bt_initialized) {
        return -1;
    }

    if (g_bt.node_count >= AGENT_BT_MAX_NODES) {
        return -1;
    }

    int32_t id = (int32_t)g_bt.node_count;
    agent_bt_node_t *n = &g_bt.nodes[id];

    n->id            = (uint32_t)id;
    n->type          = type;
    n->name          = (name != NULL) ? name : "node";
    n->tick          = tick;
    n->ctx           = ctx;
    n->child_count   = 0U;
    n->last_status   = AGENT_BT_STATUS_INVALID;
    n->tick_count    = 0U;
    n->success_count = 0U;
    n->failure_count = 0U;

    for (uint32_t i = 0U; i < AGENT_BT_MAX_CHILDREN; i++) {
        n->children[i] = -1;
    }

    g_bt.node_count++;
    return id;
}

/* ---------- ADD CHILD ---------- */
agent_status_t agent_bt_add_child(int32_t parent_id, int32_t child_id)
{
    if (parent_id < 0 || parent_id >= (int32_t)g_bt.node_count) return AGENT_ERR;
    if (child_id  < 0 || child_id  >= (int32_t)g_bt.node_count) return AGENT_ERR;

    agent_bt_node_t *p = &g_bt.nodes[parent_id];

    if (p->child_count >= AGENT_BT_MAX_CHILDREN) {
        return AGENT_ERR;
    }

    p->children[p->child_count] = child_id;
    p->child_count++;

    return AGENT_OK;
}

/* ---------- SET ROOT ---------- */
agent_status_t agent_bt_set_root(int32_t node_id)
{
    if (node_id < 0 || node_id >= (int32_t)g_bt.node_count) {
        return AGENT_ERR;
    }
    g_bt.root_id = node_id;
    return AGENT_OK;
}

/* ---------- SEQUENCE (AND) ---------- */
static agent_bt_status_t tick_sequence(agent_bt_node_t *n, uint32_t depth)
{
    if (depth > AGENT_BT_MAX_NODES) {
        return AGENT_BT_STATUS_FAILURE;
    }

    for (uint32_t i = 0U; i < n->child_count; i++) {

        int32_t cid = n->children[i];
        if (cid < 0) continue;

        agent_bt_status_t st = tick_node(cid, depth + 1U);

        if (st != AGENT_BT_STATUS_SUCCESS) {
            return st;
        }
    }
    return AGENT_BT_STATUS_SUCCESS;
}

/* ---------- SELECTOR (OR) ---------- */
static agent_bt_status_t tick_selector(agent_bt_node_t *n, uint32_t depth)
{
    if (depth > AGENT_BT_MAX_NODES) {
        return AGENT_BT_STATUS_FAILURE;
    }

    for (uint32_t i = 0U; i < n->child_count; i++) {

        int32_t cid = n->children[i];
        if (cid < 0) continue;

        agent_bt_status_t st = tick_node(cid, depth + 1U);

        if (st != AGENT_BT_STATUS_FAILURE) {
            return st;
        }
    }
    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- PARALLEL ---------- */
static agent_bt_status_t tick_parallel(agent_bt_node_t *n, uint32_t depth)
{
    if (depth > AGENT_BT_MAX_NODES) {
        return AGENT_BT_STATUS_FAILURE;
    }

    uint32_t succ = 0U;
    uint32_t fail = 0U;
    uint32_t run  = 0U;

    for (uint32_t i = 0U; i < n->child_count; i++) {

        int32_t cid = n->children[i];
        if (cid < 0) continue;

        agent_bt_status_t st = tick_node(cid, depth + 1U);

        if (st == AGENT_BT_STATUS_SUCCESS)      succ++;
        else if (st == AGENT_BT_STATUS_FAILURE) fail++;
        else                                    run++;
    }

    if (fail > 0U)                    return AGENT_BT_STATUS_FAILURE;
    if (run > 0U)                     return AGENT_BT_STATUS_RUNNING;
    if (succ == n->child_count)       return AGENT_BT_STATUS_SUCCESS;

    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- INVERTER ---------- */
static agent_bt_status_t tick_inverter(agent_bt_node_t *n, uint32_t depth)
{
    if (n->child_count < 1U) return AGENT_BT_STATUS_FAILURE;

    int32_t cid = n->children[0];
    if (cid < 0) return AGENT_BT_STATUS_FAILURE;

    agent_bt_status_t st = tick_node(cid, depth + 1U);

    if (st == AGENT_BT_STATUS_SUCCESS) return AGENT_BT_STATUS_FAILURE;
    if (st == AGENT_BT_STATUS_FAILURE) return AGENT_BT_STATUS_SUCCESS;
    return st;
}

/* ---------- REPEAT ---------- */
static agent_bt_status_t tick_repeat(agent_bt_node_t *n, uint32_t depth)
{
    if (n->child_count < 1U) return AGENT_BT_STATUS_FAILURE;

    int32_t cid = n->children[0];
    if (cid < 0) return AGENT_BT_STATUS_FAILURE;

    agent_bt_status_t st = tick_node(cid, depth + 1U);

    if (st == AGENT_BT_STATUS_SUCCESS) {
        return AGENT_BT_STATUS_RUNNING;
    }
    return st;
}

/* ---------- TICK NODE (recursive, bounded) ---------- */
static agent_bt_status_t tick_node(int32_t node_id, uint32_t depth)
{
    if (node_id < 0 || node_id >= (int32_t)g_bt.node_count) return AGENT_BT_STATUS_FAILURE;
    if (depth > AGENT_BT_MAX_NODES)                         return AGENT_BT_STATUS_FAILURE;

    agent_bt_node_t *n = &g_bt.nodes[node_id];

    g_bt.current_id = node_id;

    agent_bt_status_t st = AGENT_BT_STATUS_INVALID;

    if (n->tick != NULL) {
        st = n->tick(n->ctx);
    } else {
        switch (n->type) {
        case AGENT_BT_NODE_SEQUENCE: st = tick_sequence(n, depth); break;
        case AGENT_BT_NODE_SELECTOR: st = tick_selector(n, depth); break;
        case AGENT_BT_NODE_PARALLEL: st = tick_parallel(n, depth); break;
        case AGENT_BT_NODE_INVERTER: st = tick_inverter(n, depth); break;
        case AGENT_BT_NODE_REPEAT:   st = tick_repeat(n, depth);   break;
        default:                     st = AGENT_BT_STATUS_FAILURE; break;
        }
    }

    n->last_status = st;
    n->tick_count++;

    if (st == AGENT_BT_STATUS_SUCCESS) n->success_count++;
    if (st == AGENT_BT_STATUS_FAILURE) n->failure_count++;

    return st;
}

/* ---------- TICK (public) ---------- */
agent_bt_status_t agent_bt_tick(void)
{
    if (!g_bt_initialized)  return AGENT_BT_STATUS_INVALID;
    if (g_bt.root_id < 0)   return AGENT_BT_STATUS_INVALID;

    uint32_t t0 = agent_time_now_us();

    agent_bt_status_t st = tick_node(g_bt.root_id, 0U);

    uint32_t t1 = agent_time_now_us();
    uint32_t dt = t1 - t0;

    g_bt.total_ticks++;
    g_bt.last_tick_us = t1;
    g_bt.last_status  = st;

    if (dt > AGENT_BT_TICK_TIMEOUT_US) {
        agent_log_event(AGENT_LOG_WCET_VIOLATE, 0xB001U, dt);
    }

    return st;
}

/* ---------- QUERIES ---------- */
agent_bt_status_t agent_bt_node_status(int32_t node_id)
{
    if (node_id < 0 || node_id >= (int32_t)g_bt.node_count) {
        return AGENT_BT_STATUS_INVALID;
    }
    return g_bt.nodes[node_id].last_status;
}

int32_t agent_bt_current(void)
{
    return g_bt.current_id;
}

const agent_bt_tree_t *agent_bt_tree(void)
{
    return &g_bt;
}