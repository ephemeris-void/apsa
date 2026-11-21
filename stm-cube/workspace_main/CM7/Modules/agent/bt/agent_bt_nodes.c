#include "agent_bt_nodes.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../safety/agent_safety.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT BT NODES — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per leaf
 * ============================================================ */

static bool g_nodes_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_bt_nodes_init(void)
{
    g_nodes_initialized = true;
    return AGENT_OK;
}

/* ---------- CONDITION: health OK ---------- */
agent_bt_status_t agent_bt_node_check_health(void *ctx)
{
    (void)ctx;

    agent_health_t h = agent_health_get_system();

    if (h == AGENT_HEALTH_OK) {
        return AGENT_BT_STATUS_SUCCESS;
    }
    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- CONDITION: IMU OK ---------- */
agent_bt_status_t agent_bt_node_check_imu(void *ctx)
{
    (void)ctx;

    if (imu_health() == AGENT_HEALTH_OK) {
        return AGENT_BT_STATUS_SUCCESS;
    }
    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- CONDITION: AI OK ---------- */
agent_bt_status_t agent_bt_node_check_ai(void *ctx)
{
    (void)ctx;

    if (ai_health() == AGENT_OK) {
        return AGENT_BT_STATUS_SUCCESS;
    }
    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- CONDITION: confidence sufficient ---------- */
agent_bt_status_t agent_bt_node_check_confidence(void *ctx)
{
    (void)ctx;

    if (agent_confidence_get() >= AGENT_CONFIDENCE_MIN) {
        return AGENT_BT_STATUS_SUCCESS;
    }
    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- CONDITION: inside envelope ---------- */
agent_bt_status_t agent_bt_node_check_envelope(void *ctx)
{
    (void)ctx;

    if (agent_safety_check()) {
        return AGENT_BT_STATUS_SUCCESS;
    }
    return AGENT_BT_STATUS_FAILURE;
}

/* ---------- ACTION: hover ---------- */
agent_bt_status_t agent_bt_node_action_hover(void *ctx)
{
    (void)ctx;

    agent_context_full_t *c = agent_context_get();
    c->current_priority = AGENT_PRIO_NORMAL;

    return AGENT_BT_STATUS_SUCCESS;
}

/* ---------- ACTION: recover ---------- */
agent_bt_status_t agent_bt_node_action_recover(void *ctx)
{
    (void)ctx;

    agent_context_full_t *c = agent_context_get();
    c->current_priority = AGENT_PRIO_HIGH;

    return AGENT_BT_STATUS_SUCCESS;
}

/* ---------- ACTION: emergency ---------- */
agent_bt_status_t agent_bt_node_action_emergency(void *ctx)
{
    (void)ctx;

    agent_context_full_t *c = agent_context_get();
    c->current_priority = AGENT_PRIO_CRITICAL;

    agent_log_event(AGENT_LOG_EMERGENCY, 0xB101U, 0U);
    return AGENT_BT_STATUS_SUCCESS;
}

/* ---------- ACTION: land ---------- */
agent_bt_status_t agent_bt_node_action_land(void *ctx)
{
    (void)ctx;

    agent_context_full_t *c = agent_context_get();
    c->current_priority = AGENT_PRIO_CRITICAL;

    return AGENT_BT_STATUS_SUCCESS;
}