#ifndef AGENT_BT_NODES_H
#define AGENT_BT_NODES_H

#include "agent_bt.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT BT NODES — FROZEN
 * ============================================================
 * Standard leaf nodes (conditions + actions) for agent BT.
 * Each callback returns agent_bt_status_t.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 10 µs max per leaf.
 * ============================================================ */

agent_status_t  agent_bt_nodes_init(void);

/* ---------- Conditions ---------- */
agent_bt_status_t agent_bt_node_check_health(void *ctx);
agent_bt_status_t agent_bt_node_check_imu(void *ctx);
agent_bt_status_t agent_bt_node_check_ai(void *ctx);
agent_bt_status_t agent_bt_node_check_confidence(void *ctx);
agent_bt_status_t agent_bt_node_check_envelope(void *ctx);

/* ---------- Actions ---------- */
agent_bt_status_t agent_bt_node_action_hover(void *ctx);
agent_bt_status_t agent_bt_node_action_recover(void *ctx);
agent_bt_status_t agent_bt_node_action_emergency(void *ctx);
agent_bt_status_t agent_bt_node_action_land(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_BT_NODES_H */