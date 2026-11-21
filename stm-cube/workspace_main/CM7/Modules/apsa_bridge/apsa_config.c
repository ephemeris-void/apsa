#include "apsa_config.h"
#include <string.h>
#include <math.h>

/* ============================================================
 * APSA CONFIG — Default values
 * ============================================================
 * Board: NUCLEO-H755ZI-Q
 * ============================================================ */

/* ============================================================
 * GLOBAL INSTANCE
 * ============================================================ */
apsa_config_t g_apsa_cfg;

/* ============================================================
 * DEFAULT LOADER
 * ============================================================ */
void apsa_config_init(void)
{
    memset(&g_apsa_cfg, 0, sizeof(g_apsa_cfg));

    /* ============================================================
     * VEHICLE — 1.5 kg quadcopter, symmetric inertia
     * ============================================================ */
    g_apsa_cfg.vehicle.mass_kg       = 1.5f;
    g_apsa_cfg.vehicle.inertia_xx    = 0.020f;
    g_apsa_cfg.vehicle.inertia_yy    = 0.020f;
    g_apsa_cfg.vehicle.inertia_zz    = 0.040f;
    g_apsa_cfg.vehicle.inertia_xy    = 0.0f;
    g_apsa_cfg.vehicle.inertia_xz    = 0.0f;
    g_apsa_cfg.vehicle.inertia_yz    = 0.0f;
    g_apsa_cfg.vehicle.arm_length_m  = 0.25f;

    /* ============================================================
     * MOTORS — 4-motor X configuration
     * ============================================================ */
    g_apsa_cfg.motors.count         = 4U;
    g_apsa_cfg.motors.max_thrust_N  = 10.0f;
    g_apsa_cfg.motors.min_thrust_N  = 0.0f;
    g_apsa_cfg.motors.thrust_curve_a = 0.0f;
    g_apsa_cfg.motors.thrust_curve_b = 10.0f;

    /* Motor positions (X-config, arm 0.25 m) */
    /* M1 front-right  */
    g_apsa_cfg.motors.position[0][0] =  0.177f;
    g_apsa_cfg.motors.position[0][1] =  0.177f;
    g_apsa_cfg.motors.position[0][2] =  0.0f;

    /* M2 back-right   */
    g_apsa_cfg.motors.position[1][0] = -0.177f;
    g_apsa_cfg.motors.position[1][1] =  0.177f;
    g_apsa_cfg.motors.position[1][2] =  0.0f;

    /* M3 back-left    */
    g_apsa_cfg.motors.position[2][0] = -0.177f;
    g_apsa_cfg.motors.position[2][1] = -0.177f;
    g_apsa_cfg.motors.position[2][2] =  0.0f;

    /* M4 front-left   */
    g_apsa_cfg.motors.position[3][0] =  0.177f;
    g_apsa_cfg.motors.position[3][1] = -0.177f;
    g_apsa_cfg.motors.position[3][2] =  0.0f;

    for (uint32_t i = 0; i < 4U; ++i) {
        g_apsa_cfg.motors.direction[i][0] = 0.0f;
        g_apsa_cfg.motors.direction[i][1] = 0.0f;
        g_apsa_cfg.motors.direction[i][2] = 1.0f;

        g_apsa_cfg.motors.spin_direction[i] = (i % 2U == 0U) ? +1.0f : -1.0f;
    }

    /* ============================================================
     * AERODYNAMICS
     * ============================================================ */
    g_apsa_cfg.aero.drag_coeff         = 0.5f;
    g_apsa_cfg.aero.reference_area_m2  = 0.02f;
    g_apsa_cfg.aero.air_density        = 1.225f;
    g_apsa_cfg.aero.moment_coeff       = 0.0f;

    /* ============================================================
     * LQR — diagonal Q and R
     * ============================================================ */
    for (uint32_t i = 0; i < APSA_LQR_STATE_DIM; ++i) {
        g_apsa_cfg.lqr.q_diag[i] = 1.0f;
    }
    for (uint32_t i = 0; i < APSA_LQR_INPUT_DIM; ++i) {
        g_apsa_cfg.lqr.r_diag[i] = 0.1f;
    }
    g_apsa_cfg.lqr.max_iterations = 100U;
    g_apsa_cfg.lqr.tolerance      = 1e-10f;

    /* ============================================================
     * SMC — 2nd order surface
     * ============================================================ */
    g_apsa_cfg.smc.surface_coeffs[0] = 1.0f;
    g_apsa_cfg.smc.surface_coeffs[1] = 1.0f;
    g_apsa_cfg.smc.surface_coeffs[2] = 0.0f;
    g_apsa_cfg.smc.surface_coeffs[3] = 0.0f;
    g_apsa_cfg.smc.gain              = 0.5f;
    g_apsa_cfg.smc.phi_boundary      = 0.05f;
    g_apsa_cfg.smc.adapt_rate        = 0.1f;
    g_apsa_cfg.smc.sigma_mod         = 0.01f;

    /* ============================================================
     * MPC — horizon 20, dt 50 ms
     * ============================================================ */
    g_apsa_cfg.mpc.horizon      = 20U;
    g_apsa_cfg.mpc.dt_s         = 0.05f;
    g_apsa_cfg.mpc.max_qp_iter  = 100U;
    g_apsa_cfg.mpc.qp_tolerance = 1e-8f;

    for (uint32_t i = 0; i < APSA_MPC_STATE_DIM; ++i) {
        g_apsa_cfg.mpc.q_diag[i] = 1.0f;
    }
    for (uint32_t i = 0; i < APSA_MPC_INPUT_DIM; ++i) {
        g_apsa_cfg.mpc.r_diag[i] = 0.1f;
        g_apsa_cfg.mpc.u_min[i]  = -1.0f;
        g_apsa_cfg.mpc.u_max[i]  =  1.0f;
    }

    /* ============================================================
     * SENSOR DELAYS
     * ============================================================ */
    g_apsa_cfg.sensor.gyro_delay_s     = 0.001f;
    g_apsa_cfg.sensor.accel_delay_s    = 0.001f;
    g_apsa_cfg.sensor.baro_delay_s     = 0.010f;
    g_apsa_cfg.sensor.mag_delay_s      = 0.005f;
    g_apsa_cfg.sensor.gps_delay_s      = 0.050f;
    g_apsa_cfg.sensor.max_delay_steps  = 10U;

    /* ============================================================
     * CENTRAL BODY — Earth WGS84
     * ============================================================ */
    g_apsa_cfg.body.mu            = 3.986004418e14f;
    g_apsa_cfg.body.radius_m      = 6378137.0f;
    g_apsa_cfg.body.flattening    = 1.0f / 298.257223563f;
    g_apsa_cfg.body.j2            = 1.08262668e-3f;
    g_apsa_cfg.body.j3            = -2.53265648e-6f;
    g_apsa_cfg.body.j4            = -1.61962159e-6f;
    g_apsa_cfg.body.rotation_rate = 7.292115e-5f;

    /* ============================================================
     * ESKF NOISE — ICM-42688-P datasheet
     * ============================================================ */
    g_apsa_cfg.eskf_noise.gyro_arw       = 0.005f;
    g_apsa_cfg.eskf_noise.gyro_rrw       = 0.00001f;
    g_apsa_cfg.eskf_noise.accel_vrw      = 0.05f;
    g_apsa_cfg.eskf_noise.accel_rrw      = 0.0001f;
    g_apsa_cfg.eskf_noise.gps_pos_noise  = 1.0f;
    g_apsa_cfg.eskf_noise.gps_vel_noise  = 0.5f;
    g_apsa_cfg.eskf_noise.baro_noise     = 0.5f;
    g_apsa_cfg.eskf_noise.mag_noise      = 0.5f;
    g_apsa_cfg.eskf_noise.attitude_noise = 0.1f;   /* ~5.7° std, accelerometer-based */
}

/* ============================================================
 * VALIDATE — sanity check
 * ============================================================ */
bool apsa_config_validate(void)
{
    /* Vehicle */
    if (g_apsa_cfg.vehicle.mass_kg <= 0.0f)        { return false; }
    if (g_apsa_cfg.vehicle.inertia_xx <= 0.0f)     { return false; }
    if (g_apsa_cfg.vehicle.inertia_yy <= 0.0f)     { return false; }
    if (g_apsa_cfg.vehicle.inertia_zz <= 0.0f)     { return false; }

    /* Motors */
    if (g_apsa_cfg.motors.count == 0U)             { return false; }
    if (g_apsa_cfg.motors.count > APSA_MAX_MOTORS) { return false; }
    if (g_apsa_cfg.motors.max_thrust_N <= 0.0f)    { return false; }

    /* Aero */
    if (g_apsa_cfg.aero.air_density <= 0.0f)       { return false; }

    /* Body */
    if (g_apsa_cfg.body.mu <= 0.0f)                { return false; }
    if (g_apsa_cfg.body.radius_m <= 0.0f)          { return false; }

    /* MPC */
    if (g_apsa_cfg.mpc.horizon == 0U)              { return false; }
    if (g_apsa_cfg.mpc.dt_s <= 0.0f)               { return false; }

    /* ESKF noise */
    if (g_apsa_cfg.eskf_noise.attitude_noise <= 0.0f) { return false; }

    return true;
}

/* ============================================================
 * ACCESSOR
 * ============================================================ */
const apsa_config_t *apsa_config_get(void)
{
    return &g_apsa_cfg;
}