#include "agent_fdir_isolator.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT FDIR ISOLATOR — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per run
 * ============================================================ */

static bool g_isolator_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_fdir_isolator_init(void)
{
    g_isolator_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_fdir_isolator_reset(void)
{
    return agent_fdir_isolator_init();
}

/* ---------- Isolate IMU ---------- */
static agent_status_t isolate_imu(void)
{
    imu_on_failure();
    agent_health_set_imu(AGENT_HEALTH_FAIL);
    agent_log_event(AGENT_LOG_IMU_FAIL, 0x100U, 0U);
    return AGENT_OK;
}

/* ---------- Isolate AI ---------- */
static agent_status_t isolate_ai(void)
{
    ai_on_failure();
    agent_health_set_ai(AGENT_HEALTH_FAIL);
    agent_log_event(AGENT_LOG_AI_FAIL, 0x200U, 0U);
    return AGENT_OK;
}

/* ---------- Isolate EKF ---------- */
static agent_status_t isolate_ekf(void)
{
    ekf_on_failure();
    agent_log_event(AGENT_LOG_EKF_DIVERGE, 0x300U, 0U);
    return AGENT_OK;
}

/* ---------- APPLY ---------- */
agent_status_t agent_fdir_isolator_apply(agent_fdir_sensor_t sensor,
                                         agent_fdir_fault_t  fault)
{
    if (!g_isolator_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    switch (sensor) {

    case AGENT_FDIR_SENSOR_IMU:
        return isolate_imu();

    case AGENT_FDIR_SENSOR_AI:
        return isolate_ai();

    case AGENT_FDIR_SENSOR_EKF:
        return isolate_ekf();

    case AGENT_FDIR_SENSOR_BARO:
    case AGENT_FDIR_SENSOR_GPS:
    case AGENT_FDIR_SENSOR_MAG:
    case AGENT_FDIR_SENSOR_CONTROL:
    default:
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                        (uint32_t)sensor,
                        (uint32_t)fault);
        return AGENT_OK;
    }
}

/* ---------- RUN (channel) ---------- */
agent_status_t agent_fdir_isolator_run(agent_fdir_channel_t *ch)
{
    if (ch == NULL) {
        return AGENT_ERR;
    }

    if (ch->isolated) {
        return AGENT_OK;
    }

    agent_status_t st = agent_fdir_isolator_apply(ch->sensor, ch->fault);
    if (st != AGENT_OK) {
        return st;
    }

    ch->isolated     = true;
    ch->state        = AGENT_FDIR_STATE_ISOLATED;
    ch->isolation_us = agent_time_now_us();

    agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                    ((uint32_t)ch->sensor << 16) | (uint32_t)ch->fault,
                    ch->isolation_us);

    return AGENT_OK;
}