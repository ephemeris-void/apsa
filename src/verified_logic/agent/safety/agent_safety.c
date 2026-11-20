#include "agent_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_imu_if.h"
#include "../decisions/agent_priority.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

#include "verified_logic/sensor_layer/sal_core.h"
#include "verified_logic/sensor_layer/sal_types.h"

#include <zephyr/kernel.h>
#include <math.h>

static bool g_safety_initialized = false;
static bool g_safety_escalated   = false;
static bool g_landed             = false;

agent_status_t agent_safety_init(void)
{
    if (g_safety_initialized) return AGENT_OK;
    g_safety_escalated   = false;
    g_landed             = false;
    g_safety_initialized = true;
    agent_log_event(AGENT_LOG_SAFETY_INIT, 0U, 0U);
    return AGENT_OK;
}

bool agent_safety_check(void)
{
    if (!g_safety_initialized) return false;
    agent_trace_safety_begin();

    agent_state_t cur = agent_context_get()->current_state;
    if (cur == AGENT_STATE_VIRTUALIZE ||
        cur == AGENT_STATE_FUSION ||
        cur == AGENT_STATE_RECOVERY) {
        agent_trace_safety_end();
        return true;
    }

    agent_ekf_state_t state;
    if (ekf_get_state(&state) != AGENT_OK) {
        agent_trace_safety_end();
        return false;
    }

    float alt = state.pos[2];
    if (alt < 0.0f || alt > AGENT_ENVELOPE_ALT_MAX_M) {
        agent_log_event(AGENT_LOG_ENVELOPE_VIOLATE, 0U, (uint32_t)alt);
        agent_trace_safety_end();
        return false;
    }

    float v2 = state.vel[0]*state.vel[0] + state.vel[1]*state.vel[1] + state.vel[2]*state.vel[2];
    float vmax2 = AGENT_ENVELOPE_VEL_MAX_MS * AGENT_ENVELOPE_VEL_MAX_MS;
    if (v2 > vmax2) {
        agent_log_event(AGENT_LOG_ENVELOPE_VIOLATE, 1U, (uint32_t)v2);
        agent_trace_safety_end();
        return false;
    }

    agent_trace_safety_end();
    return true;
}

agent_status_t agent_safety_self_test(void)
{
    if (agent_context_get() == NULL) return AGENT_ERR;
    if (imu_health() != AGENT_HEALTH_OK) {
        agent_log_event(AGENT_LOG_SELFTEST_FAIL, 0U, 0U);
        return AGENT_ERR_HW;
    }
    if (ekf_health() != AGENT_OK) {
        agent_log_event(AGENT_LOG_SELFTEST_FAIL, 1U, 0U);
        return AGENT_ERR_HW;
    }
    if (!agent_safety_check()) {
        agent_log_event(AGENT_LOG_SELFTEST_FAIL, 2U, 0U);
        return AGENT_ERR;
    }
    agent_log_event(AGENT_LOG_SELFTEST_OK, 0U, 0U);
    return AGENT_OK;
}

void agent_safety_escalate(void)
{
    if (g_safety_escalated) return;
    g_safety_escalated = true;
    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0U, 0U);
    agent_context_get()->current_priority = AGENT_PRIO_CRITICAL;
    agent_fault_register(AGENT_FAULT_WCET_VIOLATE);
}

void agent_safety_deescalate(void) { g_safety_escalated = false; }

bool agent_safety_can_land(void)
{
    agent_ekf_state_t state;
    if (ekf_get_state(&state) != AGENT_OK) return false;
    if (state.pos[2] > 50.0f) return false;
    float v2 = state.vel[0]*state.vel[0] + state.vel[1]*state.vel[1] + state.vel[2]*state.vel[2];
    if (v2 > 25.0f) return false;
    float w = state.quat[0];
    if (w < 0.95f && w > -0.95f) return false;
    return true;
}

bool agent_safety_can_land_standalone(void)
{
    if (sal_get_health("baro") == SAL_HEALTH_OK) {
        sal_baro_data_t baro;
        if (sal_get_baro(&baro) == SAL_OK && baro.altitude_m > 50.0f) {
            return false;
        }
    }
    if (sal_get_health("imu") == SAL_HEALTH_OK) {
        sal_imu_data_t imu;
        if (sal_get_imu(&imu) == SAL_OK) {
            const float mg_to_ms2 = 0.004788403f;
            float ax = (float)imu.ax * mg_to_ms2;
            float ay = (float)imu.ay * mg_to_ms2;
            float az = (float)imu.az * mg_to_ms2;
            float g = sqrtf(ax*ax + ay*ay + az*az);
            if (g < 3.0f || g > 20.0f) return false;
            float tilt_cos = (g > 0.1f) ? (az / g) : 0.0f;
            if (tilt_cos < 0.85f) return false;
        }
    }
    return true;
}

bool agent_safety_is_landed(void)
{
    if (!agent_safety_can_land()) { g_landed = false; return false; }
    agent_ekf_state_t state;
    if (ekf_get_state(&state) != AGENT_OK) return false;
    if (state.pos[2] > 1.0f) return false;
    float v2 = state.vel[0]*state.vel[0] + state.vel[1]*state.vel[1] + state.vel[2]*state.vel[2];
    if (v2 > 0.25f) return false;
    g_landed = true;
    return true;
}

float agent_safety_envelope_accel_g(void)   { return AGENT_ENVELOPE_ACCEL_G; }
float agent_safety_envelope_gyro_dps(void)  { return AGENT_ENVELOPE_GYRO_DPS; }
float agent_safety_envelope_alt_max_m(void) { return AGENT_ENVELOPE_ALT_MAX_M; }
float agent_safety_envelope_vel_max_ms(void){ return AGENT_ENVELOPE_VEL_MAX_MS; }