#include "apsa_bridge.h"
#include "apsa_config.h"
#include "../apsa_modules/control_lqr.hpp"
#include "../apsa_modules/control_smc.hpp"
#include "../apsa_modules/control_sliding_extended.hpp"
#include "../apsa_modules/control_mpc.hpp"
#include "../apsa_modules/control_allocation.hpp"
#include "../apsa_modules/control_bumpless.hpp"
#include "../apsa_modules/physics_rigid_body.hpp"
#include "../apsa_modules/apsa_physics_gravity.hpp"
#include "../agent/interfaces/agent_control_if.h"
#include "../agent/core/agent_context.h"
#include "../agent/core/agent_config.h"
#include "../agent/trace/agent_log.h"
#include <string.h>
#include <math.h>

/* ============================================================
 * APSA CONTROL ADAPTER — backend for control_* interface
 * ============================================================
 * Implements agent_control_if.h using apsa_modules.
 * 25 agent files never touch apsa directly.
 * Feature flags in apsa_bridge.h gate optional controllers.
 * ============================================================ */

/* ---------- Internal state ---------- */
#if APSA_USE_LQR
static LqrResult            g_lqr;
#endif

#if APSA_USE_SMC
static SmcConfig            g_smc_cfg;
static SmcState             g_smc_state;
#endif

#if APSA_USE_MPC
static MpcConfig            g_mpc_cfg;
static MpcWorkspace         g_mpc_ws;
#endif

#if APSA_USE_ALLOCATION
static AllocationResult     g_alloc;
#endif

#if APSA_USE_BUMPLESS
static BumplessTransferState g_bumpless;
#endif

static agent_control_mode_t g_mode = AGENT_MODE_NORMAL;
static apsa_bridge_ctrl_status_t g_status;
static agent_control_out_t  g_out;

/* ============================================================
 * control_init()
 * ============================================================ */
agent_status_t control_init(void)
{
#if APSA_USE_LQR
    g_lqr = LqrResult{};
#endif

#if APSA_USE_SMC
    g_smc_cfg.system_order = 2U;
    g_smc_cfg.surface_coeffs[0] = g_apsa_cfg.smc.surface_coeffs[0];
    g_smc_cfg.surface_coeffs[1] = g_apsa_cfg.smc.surface_coeffs[1];
    g_smc_cfg.surface_coeffs[2] = g_apsa_cfg.smc.surface_coeffs[2];
    g_smc_cfg.surface_coeffs[3] = g_apsa_cfg.smc.surface_coeffs[3];
    g_smc_cfg.gain         = g_apsa_cfg.smc.gain;
    g_smc_cfg.phi_boundary = g_apsa_cfg.smc.phi_boundary;
    g_smc_state = SmcState{};
#endif

#if APSA_USE_MPC
    g_mpc_cfg = MpcConfig{};
    g_mpc_ws  = MpcWorkspace{};
#endif

#if APSA_USE_ALLOCATION
    g_alloc = AllocationResult{};
#endif

#if APSA_USE_BUMPLESS
    g_bumpless = BumplessTransferState{};
    g_bumpless.valid = false;
    g_bumpless.transfer_active = false;
#endif

    g_out = agent_control_out_t{};
    g_out.timestamp_us = 0U;

    g_mode = AGENT_MODE_NORMAL;

    g_status.iteration      = 0U;
    g_status.solve_us       = 0U;
    g_status.solve_us_max   = 0U;
    g_status.failed         = false;
    g_status.initialized    = true;

    return AGENT_OK;
}

/* ============================================================
 * Helpers — quaternion → Euler
 * ============================================================ */
static void compute_attitude_euler(const agent_ekf_state_t *state,
                                    float *roll, float *pitch, float *yaw)
{
    float qw = state->quat[0];
    float qx = state->quat[1];
    float qy = state->quat[2];
    float qz = state->quat[3];

    *roll = atan2f(2.0f * (qw * qx + qy * qz),
                   1.0f - 2.0f * (qx * qx + qy * qy));
    *pitch = asinf(2.0f * (qw * qy - qz * qx));
    *yaw = atan2f(2.0f * (qw * qz + qx * qy),
                  1.0f - 2.0f * (qy * qy + qz * qz));
}

/* ============================================================
 * Mode-specific control laws
 * ============================================================ */

/* -------- Hover -------- */
static void ctrl_hover(const agent_ekf_state_t *state)
{
    float roll, pitch, yaw;
    compute_attitude_euler(state, &roll, &pitch, &yaw);

    g_out.thrust = 9.80665f * g_apsa_cfg.vehicle.mass_kg;
    g_out.roll   = -roll  * 0.5f;
    g_out.pitch  = -pitch * 0.5f;
    g_out.yaw    = -yaw   * 0.5f;
}

/* -------- Normal -------- */
static void ctrl_normal(const agent_ekf_state_t *state)
{
    /* Position P-D → accel command */
    const float kp = 1.5f;
    const float kd = 0.8f;

    float ax = -kp * state->pos[0] - kd * state->vel[0];
    float ay = -kp * state->pos[1] - kd * state->vel[1];
    float az = -kp * state->pos[2] - kd * state->vel[2] + 9.80665f;

    float roll, pitch, yaw;
    compute_attitude_euler(state, &roll, &pitch, &yaw);

#if APSA_USE_LQR
    /* TODO: LQR position control when A/B matrices ready */
#endif

#if APSA_USE_SMC
    /* TODO: SMC attitude control when setpoint source ready */
#endif

    g_out.thrust = az * g_apsa_cfg.vehicle.mass_kg;
    g_out.roll   = -roll  * 1.0f + ay * 0.1f;
    g_out.pitch  = -pitch * 1.0f + ax * 0.1f;
    g_out.yaw    = -yaw   * 0.5f;
}

/* -------- Degraded -------- */
static void ctrl_degraded(const agent_ekf_state_t *state)
{
    ctrl_normal(state);
    g_out.thrust *= 0.95f;
}

/* -------- Emergency -------- */
static void ctrl_emergency(const agent_ekf_state_t *state)
{
    (void)state;
    g_out.thrust = 9.80665f * g_apsa_cfg.vehicle.mass_kg * 0.5f;
    g_out.roll   = 0.0f;
    g_out.pitch  = 0.0f;
    g_out.yaw    = 0.0f;
}

/* -------- Landing -------- */
static void ctrl_landing(const agent_ekf_state_t *state)
{
    float vz = state->vel[2];
    float pz = state->pos[2];

    float descent = -0.5f;
    if (pz < 1.0f) { descent = -0.2f; }

    float thrust = (9.80665f + (descent - vz) * 1.5f) *
                   g_apsa_cfg.vehicle.mass_kg;

    g_out.thrust = thrust;
    g_out.roll   = 0.0f;
    g_out.pitch  = 0.0f;
    g_out.yaw    = 0.0f;
}

/* ============================================================
 * control_update()
 * ============================================================ */
agent_status_t control_update(const agent_ekf_state_t *state)
{
    if (state == NULL)          { return AGENT_ERR; }
    if (!g_status.initialized)  { return AGENT_ERR_NOT_INIT; }
    if (g_status.failed)        { return AGENT_ERR_HW; }

    uint32_t t0 = agent_time_now_us();

    switch (g_mode) {
    case AGENT_MODE_NORMAL:     ctrl_normal(state);     break;
    case AGENT_MODE_DEGRADED:   ctrl_degraded(state);   break;
    case AGENT_MODE_VIRTUAL:    ctrl_hover(state);      break;
    case AGENT_MODE_FUSION:     ctrl_normal(state);     break;
    case AGENT_MODE_RECOVERY:   ctrl_hover(state);      break;
    case AGENT_MODE_EMERGENCY:  ctrl_emergency(state);  break;
    case AGENT_MODE_LANDING:    ctrl_landing(state);    break;
    default:                    ctrl_hover(state);      break;
    }

#if APSA_USE_BUMPLESS
    if (g_bumpless.valid && g_bumpless.transfer_active) {
        const float u_new = g_out.thrust;
        if (bumpless_update(u_new, Time{0.001f}, g_bumpless)) {
            g_out.thrust = g_bumpless.u_blended;
        }
    }
#endif

    /* Clamp */
    if (g_out.thrust < 0.0f)   { g_out.thrust = 0.0f; }
    if (g_out.thrust > 50.0f)  { g_out.thrust = 50.0f; }
    if (g_out.roll   < -1.0f)  { g_out.roll   = -1.0f; }
    if (g_out.roll   >  1.0f)  { g_out.roll   =  1.0f; }
    if (g_out.pitch  < -1.0f)  { g_out.pitch  = -1.0f; }
    if (g_out.pitch  >  1.0f)  { g_out.pitch  =  1.0f; }
    if (g_out.yaw    < -1.0f)  { g_out.yaw    = -1.0f; }
    if (g_out.yaw    >  1.0f)  { g_out.yaw    =  1.0f; }

    g_out.timestamp_us = agent_time_now_us();

    /* Timing */
    uint32_t t1 = agent_time_now_us();
    g_status.iteration++;
    g_status.solve_us = t1 - t0;
    if (g_status.solve_us > g_status.solve_us_max) {
        g_status.solve_us_max = g_status.solve_us;
    }

    if (g_status.solve_us > 500U) {
        agent_log_event(AGENT_LOG_WCET_VIOLATE, 0x300U, g_status.solve_us);
    }

    return AGENT_OK;
}

/* ============================================================
 * control_get_output()
 * ============================================================ */
agent_status_t control_get_output(agent_control_out_t *out)
{
    if (out == NULL)            { return AGENT_ERR; }
    if (!g_status.initialized)  { return AGENT_ERR_NOT_INIT; }

    *out = g_out;
    return AGENT_OK;
}

/* ============================================================
 * control_set_mode()
 * ============================================================ */
agent_status_t control_set_mode(agent_control_mode_t mode)
{
    if (mode > AGENT_MODE_LANDING) { return AGENT_ERR; }

#if APSA_USE_BUMPLESS
    if (mode != g_mode) {
        (void)bumpless_initialize(g_out.thrust, Time{0.1f}, g_bumpless);
    }
#endif

    g_mode = mode;
    return AGENT_OK;
}

/* ============================================================
 * control_health()
 * ============================================================ */
agent_status_t control_health(void)
{
    if (!g_status.initialized) { return AGENT_ERR_NOT_INIT; }
    if (g_status.failed)       { return AGENT_ERR_HW; }
    return AGENT_OK;
}

/* ============================================================
 * control_on_failure()
 * ============================================================ */
void control_on_failure(void)
{
    g_status.failed = true;
    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0x301U, 0U);
}

/* ============================================================
 * Status accessor
 * ============================================================ */
const apsa_bridge_ctrl_status_t *apsa_bridge_ctrl_status(void)
{
    return &g_status;
}

/* ============================================================
 * Bridge reset — both sides
 * ============================================================ */
void apsa_bridge_reset_all(void)
{
    (void)ekf_init();
    (void)control_init();
}