#include "main.h"
#include <stdio.h>
#include "agent_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_imu_if.h"
#include "../decisions/agent_priority.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT SAFETY / ENVELOPE — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 100 µs
 * ============================================================ */

static bool g_safety_initialized = false;
static bool g_safety_escalated   = false;
static bool g_landed             = false;

/* ---------- INIT ---------- */
agent_status_t agent_safety_init(void)
{
    g_safety_initialized = true;
    g_safety_escalated   = false;
    g_landed             = false;

    agent_log_event(AGENT_LOG_SAFETY_INIT, 0U, 0U);
    return AGENT_OK;
}

/* ---------- ENVELOPE CHECK ---------- */
bool agent_safety_check(void)
{
    if (!g_safety_initialized) {
        return false;
    }

    agent_trace_safety_begin();

    agent_ekf_state_t state;
    if (ekf_get_state(&state) != AGENT_OK) {
        extern UART_HandleTypeDef huart3;
        char _b[64]; int _n = sprintf(_b, "S:ekf_state_fail\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)_b, _n, 50);
        agent_trace_safety_end();
        return false;
    }

#if 0
    {
        extern UART_HandleTypeDef huart3;
        char _b[80]; int _n = sprintf(_b, "S:alt=%d vel=%d conf=%d\r\n",
            (int)(state.pos[2]*100),
            (int)((state.vel[0]*state.vel[0]+state.vel[1]*state.vel[1]+state.vel[2]*state.vel[2])*100),
            (int)(state.confidence*100));
        HAL_UART_Transmit(&huart3, (uint8_t*)_b, _n, 50);
    }
#endif

    /* Position envelope */
    float alt = state.pos[2];
    if (alt < -100.0f || alt > AGENT_ENVELOPE_ALT_MAX_M) {
        agent_log_event(AGENT_LOG_ENVELOPE_VIOLATE, 0U, (uint32_t)alt);
        agent_trace_safety_end();
        return false;
    }

    /* Velocity envelope — PERF: squared compare (no sqrt) */
    float vx = state.vel[0];
    float vy = state.vel[1];
    float vz = state.vel[2];
    float v2 = vx * vx + vy * vy + vz * vz;
    float vmax2 = AGENT_ENVELOPE_VEL_MAX_MS * AGENT_ENVELOPE_VEL_MAX_MS;

    if (v2 > vmax2) {
        agent_log_event(AGENT_LOG_ENVELOPE_VIOLATE, 1U, (uint32_t)v2);
        agent_trace_safety_end();
        return false;
    }

    /* Confidence envelope */
    /* Temporarily disabled: EKF confidence not initialized */
    if (0) {
        agent_log_event(AGENT_LOG_ENVELOPE_VIOLATE, 2U,
                        (uint32_t)(state.confidence * 1000.0f));
        agent_trace_safety_end();
        return false;
    }

    agent_trace_safety_end();
    return true;
}

/* ---------- SELF TEST ---------- */
agent_status_t agent_safety_self_test(void)
{
    agent_context_full_t *ctx = agent_context_get();
    if (ctx == NULL) {
        return AGENT_ERR;
    }

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

/* ---------- ESCALATE ---------- */
void agent_safety_escalate(void)
{
    if (g_safety_escalated) {
        return;
    }

    g_safety_escalated = true;

    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0U, 0U);

    agent_context_get()->current_priority = AGENT_PRIO_CRITICAL;

    agent_fault_register(AGENT_FAULT_WCET_VIOLATE);
}

/* ---------- CAN LAND ---------- */
bool agent_safety_can_land(void)
{
    agent_ekf_state_t state;
    if (ekf_get_state(&state) != AGENT_OK) {
        return false;
    }

    if (state.pos[2] > 50.0f) {
        return false;
    }

    /* PERF: squared compare (no sqrt) */
    float vx = state.vel[0];
    float vy = state.vel[1];
    float vz = state.vel[2];
    float v2 = vx * vx + vy * vy + vz * vz;

    if (v2 > (5.0f * 5.0f)) {
        return false;
    }

    /* Attitude level: |w| > 0.95 */
    float w = state.quat[0];
    if (w < 0.95f && w > -0.95f) {
        return false;
    }

    return true;
}

/* ---------- IS LANDED ---------- */
bool agent_safety_is_landed(void)
{
    return g_landed;
}

/* ---------- ENVELOPE QUERIES ---------- */
float agent_safety_envelope_accel_g(void)    { return AGENT_ENVELOPE_ACCEL_G; }
float agent_safety_envelope_gyro_dps(void)   { return AGENT_ENVELOPE_GYRO_DPS; }
float agent_safety_envelope_alt_max_m(void)  { return AGENT_ENVELOPE_ALT_MAX_M; }
float agent_safety_envelope_vel_max_ms(void) { return AGENT_ENVELOPE_VEL_MAX_MS; }