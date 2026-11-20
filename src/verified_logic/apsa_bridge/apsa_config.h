#ifndef APSA_CONFIG_H
#define APSA_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * APSA CONFIG — সব parameter-এর ঘর
 * ============================================================
 * Board: NUCLEO-H755ZI-Q
 *
 * Purpose:
 *   Single source of truth for all apsa module parameters:
 *   mass, inertia, motor geometry, LQR gains, SMC gains,
 *   MPC weights, sensor delays, gravity model, etc.
 * ============================================================ */

/* ============================================================
 * VEHICLE PHYSICAL PARAMETERS
 * ============================================================ */
typedef struct {
    float mass_kg;
    float inertia_xx;
    float inertia_yy;
    float inertia_zz;
    float inertia_xy;
    float inertia_xz;
    float inertia_yz;
    float arm_length_m;
} apsa_vehicle_t;

/* ============================================================
 * MOTOR PARAMETERS
 * ============================================================ */
#define APSA_MAX_MOTORS  8U

typedef struct {
    uint32_t count;
    float    max_thrust_N;
    float    min_thrust_N;
    float    thrust_curve_a;
    float    thrust_curve_b;
    float    position[APSA_MAX_MOTORS][3];
    float    direction[APSA_MAX_MOTORS][3];
    float    spin_direction[APSA_MAX_MOTORS];
} apsa_motors_t;

/* ============================================================
 * AERODYNAMIC PARAMETERS
 * ============================================================ */
typedef struct {
    float drag_coeff;
    float reference_area_m2;
    float air_density;
    float moment_coeff;
} apsa_aero_t;

/* ============================================================
 * LQR PARAMETERS
 * ============================================================ */
#define APSA_LQR_STATE_DIM   12U
#define APSA_LQR_INPUT_DIM   4U

typedef struct {
    float    q_diag[APSA_LQR_STATE_DIM];
    float    r_diag[APSA_LQR_INPUT_DIM];
    uint32_t max_iterations;
    float    tolerance;
    float    A[APSA_LQR_STATE_DIM][APSA_LQR_STATE_DIM];
    float    B[APSA_LQR_STATE_DIM][APSA_LQR_INPUT_DIM];
} apsa_lqr_t;

/* ============================================================
 * SMC PARAMETERS
 * ============================================================ */
#define APSA_SMC_MAX_ORDER   4U

typedef struct {
    float    surface_coeffs[APSA_SMC_MAX_ORDER];
    float    gain;
    float    phi_boundary;
    float    adapt_rate;
    float    sigma_mod;
} apsa_smc_t;

/* ============================================================
 * MPC PARAMETERS
 * ============================================================ */
#define APSA_MPC_STATE_DIM    12U
#define APSA_MPC_INPUT_DIM    4U
#define APSA_MPC_HORIZON      20U

typedef struct {
    uint32_t horizon;
    float    dt_s;
    float    q_diag[APSA_MPC_STATE_DIM];
    float    r_diag[APSA_MPC_INPUT_DIM];
    float    u_min[APSA_MPC_INPUT_DIM];
    float    u_max[APSA_MPC_INPUT_DIM];
    uint32_t max_qp_iter;
    float    qp_tolerance;
} apsa_mpc_t;

/* ============================================================
 * SENSOR DELAYS
 * ============================================================ */
typedef struct {
    float gyro_delay_s;
    float accel_delay_s;
    float baro_delay_s;
    float mag_delay_s;
    float gps_delay_s;
    uint32_t max_delay_steps;
} apsa_sensor_delay_t;

/* ============================================================
 * CENTRAL BODY (GRAVITY)
 * ============================================================ */
typedef struct {
    float mu;
    float radius_m;
    float flattening;
    float j2;
    float j3;
    float j4;
    float rotation_rate;
} apsa_body_t;

/* ============================================================
 * ESKF NOISE PARAMETERS
 * ============================================================ */
typedef struct {
    float gyro_arw;             /* angle random walk */
    float gyro_rrw;             /* rate random walk */
    float accel_vrw;            /* velocity random walk */
    float accel_rrw;            /* rate random walk */
    float gps_pos_noise;
    float gps_vel_noise;
    float baro_noise;
    float mag_noise;
    float attitude_noise;       /* accelerometer-based attitude measurement noise (rad) */
} apsa_eskf_noise_t;

/* ============================================================
 * MASTER CONFIG
 * ============================================================ */
typedef struct {
    apsa_vehicle_t      vehicle;
    apsa_motors_t       motors;
    apsa_aero_t         aero;
    apsa_lqr_t          lqr;
    apsa_smc_t          smc;
    apsa_mpc_t          mpc;
    apsa_sensor_delay_t sensor;
    apsa_body_t         body;
    apsa_eskf_noise_t   eskf_noise;
} apsa_config_t;

/* ============================================================
 * GLOBAL CONFIG INSTANCE
 * ============================================================ */
extern apsa_config_t g_apsa_cfg;

/* ============================================================
 * CONFIG API
 * ============================================================ */
void     apsa_config_init(void);
bool     apsa_config_validate(void);

const apsa_config_t *apsa_config_get(void);

#ifdef __cplusplus
}
#endif

#endif /* APSA_CONFIG_H */