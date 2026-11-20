#include "apsa_bridge.h"
#include "apsa_config.h"
#include "../../apsa_modules/control_lqr.hpp"
#include "../../apsa_modules/control_smc.hpp"
#include "../../apsa_modules/control_bumpless.hpp"
#include "../../agent/interfaces/agent_control_if.h"
#include "../../agent/core/agent_context.h"
#include "../../agent/core/agent_config.h"
#include "../../agent/trace/agent_log.h"
#include <string.h>
#include <math.h>

#if APSA_USE_LQR
static LqrResult            g_lqr;
static bool                 g_lqr_initialized = false;
static Matrix_Dynamic<kLqrMaxState, kLqrMaxState> g_lqr_A;
static Matrix_Dynamic<kLqrMaxState, kLqrMaxControl> g_lqr_B;
static Matrix_Dynamic<kLqrMaxState, kLqrMaxState> g_lqr_Q;
static Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl> g_lqr_R;
#endif

#if APSA_USE_SMC
static SmcConfig            g_smc_cfg;
static SmcState             g_smc_state;
#endif

#if APSA_USE_BUMPLESS
static BumplessTransferState g_bumpless;
#endif

static agent_control_mode_t g_mode = AGENT_MODE_NORMAL;
static apsa_bridge_ctrl_status_t g_status;
static agent_control_out_t  g_out;

agent_status_t control_init(void)
{
#if APSA_USE_LQR
    g_lqr_A = Matrix_Dynamic<kLqrMaxState, kLqrMaxState>{APSA_LQR_STATE_DIM, APSA_LQR_STATE_DIM};
    g_lqr_B = Matrix_Dynamic<kLqrMaxState, kLqrMaxControl>{APSA_LQR_STATE_DIM, APSA_LQR_INPUT_DIM};
    g_lqr_Q = Matrix_Dynamic<kLqrMaxState, kLqrMaxState>{APSA_LQR_STATE_DIM, APSA_LQR_STATE_DIM};
    g_lqr_R = Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl>{APSA_LQR_INPUT_DIM, APSA_LQR_INPUT_DIM};

    for (uint32_t i = 0; i < APSA_LQR_STATE_DIM; ++i) {
        for (uint32_t j = 0; j < APSA_LQR_STATE_DIM; ++j) {
            g_lqr_A.at(i, j) = g_apsa_cfg.lqr.A[i][j];
            g_lqr_Q.at(i, j) = (i == j) ? g_apsa_cfg.lqr.q_diag[i] : 0.0f;
        }
    }
    for (uint32_t i = 0; i < APSA_LQR_STATE_DIM; ++i) {
        for (uint32_t j = 0; j < APSA_LQR_INPUT_DIM; ++j) {
            g_lqr_B.at(i, j) = g_apsa_cfg.lqr.B[i][j];
        }
    }
    for (uint32_t i = 0; i < APSA_LQR_INPUT_DIM; ++i) {
        for (uint32_t j = 0; j < APSA_LQR_INPUT_DIM; ++j) {
            g_lqr_R.at(i, j) = (i == j) ? g_apsa_cfg.lqr.r_diag[i] : 0.0f;
        }
    }

    g_lqr = LqrResult{};
    g_lqr_initialized = lqr_solve_dare(g_lqr_A, g_lqr_B, g_lqr_Q, g_lqr_R, g_lqr);
    if (!g_lqr_initialized) {
        agent_log_event(AGENT_LOG_SELFTEST_FAIL, 0x400U, 0U);
    }
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

#if APSA_USE_LQR
static bool ctrl_lqr_attitude(const agent_ekf_state_t *state)
{
    if (!g_lqr_initialized) return false;

    Vector_Dynamic<kLqrMaxState> x{APSA_LQR_STATE_DIM};
    for (uint32_t i = 0; i < APSA_LQR_STATE_DIM; ++i) {
        x.at(i) = 0.0f;
    }

    float roll, pitch, yaw;
    compute_attitude_euler(state, &roll, &pitch, &yaw);

    x.at(0) = roll;
    x.at(1) = pitch;
    x.at(2) = yaw;

    Vector_Dynamic<kLqrMaxControl> u{APSA_LQR_INPUT_DIM};
    if (!lqr_control(g_lqr.K, x, u)) {
        return false;
    }

    g_out.roll  = u.at(0);
    g_out.pitch = u.at(1);
    g_out.yaw   = u.at(2);
    g_out.thrust = g_apsa_cfg.vehicle.mass_kg * 9.80665f;

    return true;
}
#endif

#if APSA_USE_SMC
static bool ctrl_smc_attitude(const agent_ekf_state_t *state)
{
    float roll, pitch, yaw;
    compute_attitude_euler(state, &roll, &pitch, &yaw);

    scalar u_roll = 0.0f, u_pitch = 0.0f, u_yaw = 0.0f;
    bool ok = true;

    ok = ok && smc_control(g_smc_cfg, g_smc_state, -roll, 0.0f, u_roll);
    ok = ok && smc_control(g_smc_cfg, g_smc_state, -pitch, 0.0f, u_pitch);
    ok = ok && smc_control(g_smc_cfg, g_smc_state, -yaw, 0.0f, u_yaw);

    if (!ok) return false;

    g_out.roll  = u_roll;
    g_out.pitch = u_pitch;
    g_out.yaw   = u_yaw;
    g_out.thrust = g_apsa_cfg.vehicle.mass_kg * 9.80665f;

    return true;
}
#endif

static void ctrl_hover(const agent_ekf_state_t *state)
{
    float roll, pitch, yaw;
    compute_attitude_euler(state, &roll, &pitch, &yaw);

    g_out.thrust = g_apsa_cfg.vehicle.mass_kg * 9.80665f;
    g_out.roll   = -roll  * 0.5f;
    g_out.pitch  = -pitch * 0.5f;
    g_out.yaw    = -yaw   * 0.5f;
}

static void ctrl_normal(const agent_ekf_state_t *state)
{
    bool controlled = false;

#if APSA_USE_LQR
    controlled = ctrl_lqr_attitude(state);
#endif

#if APSA_USE_SMC
    if (!controlled) {
        controlled = ctrl_smc_attitude(state);
    }
#endif

    if (!controlled) {
        g_out.roll   = 0.0f;
        g_out.pitch  = 0.0f;
        g_out.yaw    = 0.0f;
        g_out.thrust = 0.0f;
    }
}

static void ctrl_degraded(const agent_ekf_state_t *state)
{
    ctrl_normal(state);
    g_out.thrust *= 0.95f;
}

static void ctrl_emergency(const agent_ekf_state_t *state)
{
    (void)state;
    g_out.thrust = g_apsa_cfg.vehicle.mass_kg * 9.80665f * 0.5f;
    g_out.roll   = 0.0f;
    g_out.pitch  = 0.0f;
    g_out.yaw    = 0.0f;
}

static void ctrl_landing(const agent_ekf_state_t *state)
{
    float vz = state->vel[2];
    float pz = state->pos[2];

    float descent = -0.5f;
    if (pz < 1.0f) { descent = -0.2f; }

    g_out.thrust = (g_apsa_cfg.vehicle.mass_kg * 9.80665f) + 
                   (descent - vz) * g_apsa_cfg.vehicle.mass_kg * 1.5f;
    g_out.roll   = 0.0f;
    g_out.pitch  = 0.0f;
    g_out.yaw    = 0.0f;
}

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

    if (g_out.thrust < 0.0f)   { g_out.thrust = 0.0f; }
    if (g_out.thrust > 50.0f)  { g_out.thrust = 50.0f; }
    if (g_out.roll   < -1.0f)  { g_out.roll   = -1.0f; }
    if (g_out.roll   >  1.0f)  { g_out.roll   =  1.0f; }
    if (g_out.pitch  < -1.0f)  { g_out.pitch  = -1.0f; }
    if (g_out.pitch  >  1.0f)  { g_out.pitch  =  1.0f; }
    if (g_out.yaw    < -1.0f)  { g_out.yaw    = -1.0f; }
    if (g_out.yaw    >  1.0f)  { g_out.yaw    =  1.0f; }

    g_out.timestamp_us = agent_time_now_us();

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

agent_status_t control_get_output(agent_control_out_t *out)
{
    if (out == NULL)            { return AGENT_ERR; }
    if (!g_status.initialized)  { return AGENT_ERR_NOT_INIT; }

    *out = g_out;
    return AGENT_OK;
}

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

agent_status_t control_health(void)
{
    if (!g_status.initialized) { return AGENT_ERR_NOT_INIT; }
    if (g_status.failed)       { return AGENT_ERR_HW; }
    return AGENT_OK;
}

void control_on_failure(void)
{
    g_status.failed = true;
    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0x301U, 0U);
}

const apsa_bridge_ctrl_status_t *apsa_bridge_ctrl_status(void)
{
    return &g_status;
}

void apsa_bridge_reset_all(void)
{
    (void)ekf_init();
    (void)control_init();
}