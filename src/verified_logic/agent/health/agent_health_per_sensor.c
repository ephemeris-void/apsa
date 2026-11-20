#include "agent_health_per_sensor.h"
#include "agent_health.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_control_if.h"

/* ============================================================
 * AGENT HEALTH — PER-SENSOR POLLING IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 25 µs per poll (all sensors)
 * ============================================================ */

static bool g_per_sensor_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_health_per_sensor_init(void)
{
    g_per_sensor_initialized = true;
    return AGENT_OK;
}

/* ---------- POLL IMU ---------- */
agent_status_t agent_health_poll_imu(void)
{
    if (!g_per_sensor_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_health_t h = imu_health();
    return agent_health_set(AGENT_HEALTH_SENSOR_IMU, h);
}

/* ---------- POLL AI ---------- */
agent_status_t agent_health_poll_ai(void)
{
    if (!g_per_sensor_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_status_t st = ai_health();
    agent_health_t h  = (st == AGENT_OK) ? AGENT_HEALTH_OK : AGENT_HEALTH_FAIL;
    return agent_health_set(AGENT_HEALTH_SENSOR_AI, h);
}

/* ---------- POLL EKF ---------- */
agent_status_t agent_health_poll_ekf(void)
{
    if (!g_per_sensor_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_status_t st = ekf_health();
    agent_health_t h  = (st == AGENT_OK) ? AGENT_HEALTH_OK : AGENT_HEALTH_FAIL;
    return agent_health_set(AGENT_HEALTH_SENSOR_EKF, h);
}

/* ---------- POLL CONTROL ---------- */
agent_status_t agent_health_poll_control(void)
{
    if (!g_per_sensor_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_status_t st = control_health();
    agent_health_t h  = (st == AGENT_OK) ? AGENT_HEALTH_OK : AGENT_HEALTH_FAIL;
    return agent_health_set(AGENT_HEALTH_SENSOR_CONTROL, h);
}

/* ---------- POLL ALL ---------- */
agent_status_t agent_health_poll_all(void)
{
    (void)agent_health_poll_imu();
    (void)agent_health_poll_ai();
    (void)agent_health_poll_ekf();
    (void)agent_health_poll_control();
    (void)agent_health_update();
    return AGENT_OK;
}