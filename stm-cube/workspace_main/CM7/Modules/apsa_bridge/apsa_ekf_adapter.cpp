#include "apsa_bridge.h"
#include "apsa_config.h"
#include "../apsa_modules/estimation_eskf.hpp"
#include "../apsa_modules/estimation_ekf.hpp"
#include "../apsa_modules/estimation_ukf.hpp"
#include "../apsa_modules/estimation_innovation_monitor.hpp"
#include "../apsa_modules/estimation_sensor_delay.hpp"
#include "../apsa_modules/estimation_types.hpp"
#include "../apsa_modules/apsa_frame_system.hpp"
#include "../apsa_modules/apsa_physics_math.hpp"
#include "../apsa_modules/apsa_physics_gravity.hpp"
#include "../agent/interfaces/agent_ekf_if.h"
#include "../agent/interfaces/agent_imu_if.h"
#include "../agent/core/agent_context.h"
#include "../agent/core/agent_config.h"
#include "../agent/trace/agent_log.h"
#include <string.h>
#include <cmath>

/* ============================================================
 * APSA EKF ADAPTER — POSITION ACCURACY FIXED
 * ============================================================
 * Changes:
 *   1. Gyro clamp (±2000 dps)
 *   2. Attitude flip detection + auto reset
 *   3. Accelerometer measurement update (10x gain)
 *   4. ZUPT attitude hold (relaxed motion detection)
 *   5. Wire disconnect position freeze
 *   6. NEW: Velocity leak + position hold in ZUPT (high accuracy)
 * ============================================================ */

#if APSA_USE_ESKF
static EskfNominalState           g_nominal;
static EskfErrorState             g_eskf;
static EskfWorkspace              g_ws;
static EskfProcessNoiseConfig     g_proc_noise;
static EskfMeasurementNoiseConfig g_meas_noise;

/* ZUPT state */
static uint32_t                   g_static_count = 0U;

/* Accelerometer measurement update state */
static uint32_t                   g_accel_update_count = 0U;

/* Position hold state during sensor fail */
static bool                       g_position_held = false;
#endif

#if APSA_USE_INNOVATION_MON
static ImuAxisNisState            g_nis;
static ImuHealthState             g_health;
static ImuHealthSmoother          g_smoother;
#endif

#if APSA_USE_SENSOR_DELAY
static SensorDelayConfig          g_delay_cfg;
static SensorDelayState           g_delay_state;
static SensorDelayWorkspace       g_delay_ws;
#endif

static apsa_bridge_ekf_status_t   g_status;

#define BRIDGE_MG_TO_MS2     (9.80665f / 2048.0f)
#define BRIDGE_DPS_TO_RADS   (0.0174532925f / 16.4f)
#define BRIDGE_GRAVITY       (9.80665f)

/* ---------- ZUPT thresholds (relaxed for small motion) ---------- */
#define BRIDGE_ZUPT_ACCEL_TOL    (1.0f)
#define BRIDGE_ZUPT_GYRO_TOL     (0.1f)
#define BRIDGE_ZUPT_MIN_SAMPLES  (50U)

/* ---------- Accelerometer measurement update thresholds ---------- */
#define BRIDGE_ACCEL_UPDATE_DECIMATION  (10U)

/* ---------- Gyro clamp threshold (±2000 dps = ±3276 raw) ---------- */
#define BRIDGE_GYRO_CLAMP_RAW    (3276)

/* ---------- Attitude flip detection threshold ---------- */
#define BRIDGE_ATTITUDE_FLIP_W   (0.3f)

/* ---------- Position accuracy: velocity leak + position hold ---------- */
#define BRIDGE_VELOCITY_LEAK     (0.0001f)   /* Slow velocity decay per ms */

/* ============================================================
 * ekf_init()
 * ============================================================ */
agent_status_t ekf_init(void)
{
#if APSA_USE_ESKF
    g_nominal = EskfNominalState{};
    g_nominal.pos_x     = Length{0.0f};
    g_nominal.pos_y     = Length{0.0f};
    g_nominal.pos_z     = Length{0.0f};
    g_nominal.vel_x     = Velocity{0.0f};
    g_nominal.vel_y     = Velocity{0.0f};
    g_nominal.vel_z     = Velocity{0.0f};
    g_nominal.attitude  = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    g_nominal.bias      = EskfImuBias{};

    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> P0{kEskfStateDim, kEskfStateDim};

    for (std::size_t i = 0U; i < kEskfStateDim; ++i) {
        for (std::size_t j = 0U; j < kEskfStateDim; ++j) {
            P0.at(i, j) = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (std::size_t i = 0U; i < 3U; ++i) {
        P0.at(0U  + i, 0U  + i) = 0.1f;
        P0.at(3U  + i, 3U  + i) = 0.1f;
        P0.at(6U  + i, 6U  + i) = 0.01f;
        P0.at(9U  + i, 9U  + i) = 0.001f;
        P0.at(12U + i, 12U + i) = 0.001f;
    }

    if (!eskf_initialize(g_nominal, P0, g_eskf)) {
        agent_log_event(AGENT_LOG_EKF_DIVERGE, 0xE001U, 0U);
        return AGENT_ERR;
    }

    g_proc_noise.gyro_arw  = g_apsa_cfg.eskf_noise.gyro_arw;
    g_proc_noise.gyro_rrw  = g_apsa_cfg.eskf_noise.gyro_rrw;
    g_proc_noise.accel_vrw = g_apsa_cfg.eskf_noise.accel_vrw;
    g_proc_noise.accel_rrw = g_apsa_cfg.eskf_noise.accel_rrw;
    g_proc_noise.pos_rw    = 0.001f;
    g_proc_noise.vel_rw    = 0.001f;

    g_meas_noise.gps_pos = g_apsa_cfg.eskf_noise.gps_pos_noise;
    g_meas_noise.gps_vel = g_apsa_cfg.eskf_noise.gps_vel_noise;

    g_static_count        = 0U;
    g_accel_update_count  = 0U;
    g_position_held       = false;
#endif

#if APSA_USE_INNOVATION_MON
    g_nis = ImuAxisNisState{};
    g_health = ImuHealthState{};
    g_smoother = ImuHealthSmoother{};
    g_smoother.alpha = 0.1f;
    g_smoother.initialized = false;
#endif

#if APSA_USE_SENSOR_DELAY
    g_delay_cfg.known_delay      = Time{0.0f};
    g_delay_cfg.sample_period    = Time{0.001f};
    g_delay_cfg.max_delay_steps  = g_apsa_cfg.sensor.max_delay_steps;
    g_delay_state = SensorDelayState{};
#endif

    g_status.update_count   = 0U;
    g_status.last_update_us = agent_time_now_us();
    g_status.confidence     = 0.0f;
    g_status.diverged       = false;
    g_status.initialized    = true;

    return AGENT_OK;
}

/* ============================================================
 * ekf_set_initial_attitude_from_accel()
 * ============================================================ */
void ekf_set_initial_attitude_from_accel(float ax_ms2, float ay_ms2, float az_ms2)
{
#if APSA_USE_ESKF
    float roll  = atan2f(ay_ms2, az_ms2);
    float pitch = atan2f(-ax_ms2, sqrtf(ay_ms2 * ay_ms2 + az_ms2 * az_ms2));

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    g_nominal.attitude = Quaternion{
        cr * cp,
        sr * cp,
        cr * sp,
        -sr * sp
    };

    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> P0{kEskfStateDim, kEskfStateDim};

    for (std::size_t i = 0U; i < kEskfStateDim; ++i) {
        for (std::size_t j = 0U; j < kEskfStateDim; ++j) {
            P0.at(i, j) = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (std::size_t i = 0U; i < 3U; ++i) {
        P0.at(0U  + i, 0U  + i) = 0.1f;
        P0.at(3U  + i, 3U  + i) = 0.1f;
        P0.at(6U  + i, 6U  + i) = 0.01f;
        P0.at(9U  + i, 9U  + i) = 0.001f;
        P0.at(12U + i, 12U + i) = 0.001f;
    }

    (void)eskf_initialize(g_nominal, P0, g_eskf);

    g_nominal.pos_x = Length{0.0f};
    g_nominal.pos_y = Length{0.0f};
    g_nominal.pos_z = Length{0.0f};
    g_nominal.vel_x = Velocity{0.0f};
    g_nominal.vel_y = Velocity{0.0f};
    g_nominal.vel_z = Velocity{0.0f};

    g_static_count       = 0U;
    g_accel_update_count = 0U;
    g_position_held      = false;

    g_status.update_count   = 0U;
    g_status.last_update_us = agent_time_now_us();
    g_status.diverged       = false;
    g_status.confidence     = 1.0f;
#endif
}

/* ============================================================
 * ekf_update()
 * ============================================================ */
agent_status_t ekf_update(const agent_sensor_sample_t *sample)
{
    if (sample == NULL)              { return AGENT_ERR; }
    if (!g_status.initialized)       { return AGENT_ERR_NOT_INIT; }
    if (g_status.diverged) {
        uint32_t now_r = agent_time_now_us();
        if ((now_r - g_status.last_update_us) > 5000000U) {
            ekf_init();
            g_status.last_update_us = now_r;
        }
        return AGENT_ERR_HW;
    }

#if APSA_USE_ESKF
    if (sample->ax == -1 && sample->ay == -1 && sample->az == -1 &&
        sample->gx == -1 && sample->gy == -1 && sample->gz == -1) {
        return AGENT_OK;
    }

    /* ---------- FIX: Wire Disconnect Position Freeze ---------- */
    if (imu_health() != AGENT_HEALTH_OK) {
        /* Force velocity to zero — position hold */
        g_nominal.vel_x = Velocity{0.0f};
        g_nominal.vel_y = Velocity{0.0f};
        g_nominal.vel_z = Velocity{0.0f};

        g_position_held = true;

        g_status.update_count++;
        g_status.last_update_us = agent_time_now_us();
        return AGENT_OK;
    } else {
        g_position_held = false;
    }

    uint32_t now   = agent_time_now_us();
    uint32_t dt_us = now - g_status.last_update_us;
    float    dt_s  = (float)dt_us * 1e-6f;

    if (dt_s < 1e-4f) { dt_s = 1e-4f; }
    if (dt_s > 5e-2f) { dt_s = 5e-2f; }

    /* ---------- FIX 1: Gyro Clamp (±2000 dps) ---------- */
    int16_t gx_raw = sample->gx;
    int16_t gy_raw = sample->gy;
    int16_t gz_raw = sample->gz;

    if (gx_raw >  BRIDGE_GYRO_CLAMP_RAW) gx_raw =  BRIDGE_GYRO_CLAMP_RAW;
    if (gx_raw < -BRIDGE_GYRO_CLAMP_RAW) gx_raw = -BRIDGE_GYRO_CLAMP_RAW;
    if (gy_raw >  BRIDGE_GYRO_CLAMP_RAW) gy_raw =  BRIDGE_GYRO_CLAMP_RAW;
    if (gy_raw < -BRIDGE_GYRO_CLAMP_RAW) gy_raw = -BRIDGE_GYRO_CLAMP_RAW;
    if (gz_raw >  BRIDGE_GYRO_CLAMP_RAW) gz_raw =  BRIDGE_GYRO_CLAMP_RAW;
    if (gz_raw < -BRIDGE_GYRO_CLAMP_RAW) gz_raw = -BRIDGE_GYRO_CLAMP_RAW;

    AngularVel gx{BRIDGE_DPS_TO_RADS * (float)gx_raw};
    AngularVel gy{BRIDGE_DPS_TO_RADS * (float)gy_raw};
    AngularVel gz{BRIDGE_DPS_TO_RADS * (float)gz_raw};

    Acceleration ax{BRIDGE_MG_TO_MS2 * (float)sample->ax};
    Acceleration ay{BRIDGE_MG_TO_MS2 * (float)sample->ay};
    Acceleration az{BRIDGE_MG_TO_MS2 * (float)sample->az};

    Time dt{dt_s};

    if (!eskf_predict(g_nominal, gx, gy, gz, ax, ay, az, dt,
                      g_proc_noise, g_eskf, g_ws)) {
        g_status.diverged = true;
        agent_log_event(AGENT_LOG_EKF_DIVERGE, 0xE002U, g_status.update_count);
        return AGENT_ERR_HW;
    }

    /* Attitude update (gyro integration) */
    {
        float half_dt = 0.5f * dt_s;
        float dqx = gx.get() * half_dt;
        float dqy = gy.get() * half_dt;
        float dqz = gz.get() * half_dt;

        float dq_norm = sqrtf(1.0f + dqx*dqx + dqy*dqy + dqz*dqz);
        if (dq_norm > 1e-6f) {
            Quaternion dq{1.0f / dq_norm,
                          dqx / dq_norm,
                          dqy / dq_norm,
                          dqz / dq_norm};

            Quaternion q_new = g_nominal.attitude * dq;

            float qn = sqrtf(q_new.get_w() * q_new.get_w() +
                             q_new.get_x() * q_new.get_x() +
                             q_new.get_y() * q_new.get_y() +
                             q_new.get_z() * q_new.get_z());
            if (qn > 1e-6f) {
                g_nominal.attitude = Quaternion{
                    q_new.get_w() / qn,
                    q_new.get_x() / qn,
                    q_new.get_y() / qn,
                    q_new.get_z() / qn
                };
            }
        }
    }

    /* ---------- FIX 2: Attitude Flip Detection ---------- */
    {
        float qw_check = g_nominal.attitude.get_w();
        if (fabsf(qw_check) < BRIDGE_ATTITUDE_FLIP_W) {
            float ax_g = ax.get();
            float ay_g = ay.get();
            float az_g = az.get();
            
            ekf_set_initial_attitude_from_accel(ax_g, ay_g, az_g);
            agent_log_event(AGENT_LOG_EKF_RESET, 0x7777U, g_status.update_count);
            
            g_status.update_count++;
            g_status.last_update_us = now;
            return AGENT_OK;
        }
    }

    /* Gravity removal + world-frame linear accel */
    {
        float qw = g_nominal.attitude.get_w();
        float qx = g_nominal.attitude.get_x();
        float qy = g_nominal.attitude.get_y();
        float qz = g_nominal.attitude.get_z();

        float r00 = 1.0f - 2.0f * (qy*qy + qz*qz);
        float r01 = 2.0f * (qx*qy - qw*qz);
        float r02 = 2.0f * (qx*qz + qw*qy);
        float r10 = 2.0f * (qx*qy + qw*qz);
        float r11 = 1.0f - 2.0f * (qx*qx + qz*qz);
        float r12 = 2.0f * (qy*qz - qw*qx);
        float r20 = 2.0f * (qx*qz - qw*qy);
        float r21 = 2.0f * (qy*qz + qw*qx);
        float r22 = 1.0f - 2.0f * (qx*qx + qy*qy);

        float g_body_x = r20 * BRIDGE_GRAVITY;
        float g_body_y = r21 * BRIDGE_GRAVITY;
        float g_body_z = r22 * BRIDGE_GRAVITY;

        float a_lin_body_x = ax.get() - g_body_x;
        float a_lin_body_y = ay.get() - g_body_y;
        float a_lin_body_z = az.get() - g_body_z;

        float a_lin_world_x = r00 * a_lin_body_x + r01 * a_lin_body_y + r02 * a_lin_body_z;
        float a_lin_world_y = r10 * a_lin_body_x + r11 * a_lin_body_y + r12 * a_lin_body_z;
        float a_lin_world_z = r20 * a_lin_body_x + r21 * a_lin_body_y + r22 * a_lin_body_z;

        /* ---------- FIX 4: ZUPT (Relaxed Motion Detection) ---------- */
        float accel_mag = sqrtf(ax.get()*ax.get() +
                                ay.get()*ay.get() +
                                az.get()*az.get());

        float gyro_mag  = sqrtf(gx.get()*gx.get() +
                                gy.get()*gy.get() +
                                gz.get()*gz.get());

        bool static_condition = (fabsf(accel_mag - BRIDGE_GRAVITY) < BRIDGE_ZUPT_ACCEL_TOL) &&
                                (gyro_mag < BRIDGE_ZUPT_GYRO_TOL);

        if (static_condition) {
            g_static_count++;
        } else {
            g_static_count = 0U;
        }

        if (g_static_count >= BRIDGE_ZUPT_MIN_SAMPLES) {
            /* Static: force velocity to zero */
            g_nominal.vel_x = Velocity{0.0f};
            g_nominal.vel_y = Velocity{0.0f};
            g_nominal.vel_z = Velocity{0.0f};

            /* ZUPT Attitude Hold — force level from accel */
            float roll_acc  = atan2f(ay.get(), az.get());
            float pitch_acc = atan2f(-ax.get(), sqrtf(ay.get()*ay.get() + az.get()*az.get()));

            float cr = cosf(roll_acc * 0.5f);
            float sr = sinf(roll_acc * 0.5f);
            float cp = cosf(pitch_acc * 0.5f);
            float sp = sinf(pitch_acc * 0.5f);

            g_nominal.attitude = Quaternion{
                cr * cp,
                sr * cp,
                cr * sp,
                -sr * sp
            };

            /* ---------- POSITION ACCURACY FIX ----------
             * During static, force position hold (no integration).
             * This prevents position drift when board is stationary.
             */
            /* Position NOT integrated — held at last value */
        } else {
            /* Dynamic: normal integration with velocity leak */

            /* Velocity leak — slow decay to prevent drift accumulation */
            float vx = g_nominal.vel_x.get();
            float vy = g_nominal.vel_y.get();
            float vz = g_nominal.vel_z.get();

            vx *= (1.0f - BRIDGE_VELOCITY_LEAK);
            vy *= (1.0f - BRIDGE_VELOCITY_LEAK);
            vz *= (1.0f - BRIDGE_VELOCITY_LEAK);

            vx += a_lin_world_x * dt_s;
            vy += a_lin_world_y * dt_s;
            vz += a_lin_world_z * dt_s;

            g_nominal.vel_x = Velocity{vx};
            g_nominal.vel_y = Velocity{vy};
            g_nominal.vel_z = Velocity{vz};

            /* Position integration with velocity clamp */
            float pos_x = g_nominal.pos_x.get() + g_nominal.vel_x.get() * dt_s;
            float pos_y = g_nominal.pos_y.get() + g_nominal.vel_y.get() * dt_s;
            float pos_z = g_nominal.pos_z.get() + g_nominal.vel_z.get() * dt_s;

            g_nominal.pos_x = Length{pos_x};
            g_nominal.pos_y = Length{pos_y};
            g_nominal.pos_z = Length{pos_z};
        }
    }

    /* ---------- FIX 3: Accelerometer Measurement Update (10x gain) ---------- */
    g_accel_update_count++;
    if (g_accel_update_count >= BRIDGE_ACCEL_UPDATE_DECIMATION) {
        g_accel_update_count = 0U;

        float qw = g_nominal.attitude.get_w();
        float qx = g_nominal.attitude.get_x();
        float qy = g_nominal.attitude.get_y();
        float qz = g_nominal.attitude.get_z();

        float r20 = 2.0f * (qx*qz - qw*qy);
        float r21 = 2.0f * (qy*qz + qw*qx);
        float r22 = 1.0f - 2.0f * (qx*qx + qy*qy);

        float h_x = -r20 * BRIDGE_GRAVITY;
        float h_y = -r21 * BRIDGE_GRAVITY;
        float h_z = -r22 * BRIDGE_GRAVITY;

        float vn = sqrtf(h_x*h_x + h_y*h_y + h_z*h_z);
        if (vn > 1e-6f) {
            float ex = (ay.get() * h_z - az.get() * h_y) / (vn * vn);
            float ey = (az.get() * h_x - ax.get() * h_z) / (vn * vn);
            float ez = (ax.get() * h_y - ay.get() * h_x) / (vn * vn);

            float gain = 0.5f * 0.01f;

            float dqx = 0.5f * gain * ex;
            float dqy = 0.5f * gain * ey;
            float dqz = 0.5f * gain * ez;

            float dq_norm = sqrtf(1.0f + dqx*dqx + dqy*dqy + dqz*dqz);
            if (dq_norm > 1e-6f) {
                Quaternion dq{1.0f / dq_norm,
                              dqx / dq_norm,
                              dqy / dq_norm,
                              dqz / dq_norm};

                Quaternion q_new = g_nominal.attitude * dq;

                float qn = sqrtf(q_new.get_w() * q_new.get_w() +
                                 q_new.get_x() * q_new.get_x() +
                                 q_new.get_y() * q_new.get_y() +
                                 q_new.get_z() * q_new.get_z());
                if (qn > 1e-6f) {
                    g_nominal.attitude = Quaternion{
                        q_new.get_w() / qn,
                        q_new.get_x() / qn,
                        q_new.get_y() / qn,
                        q_new.get_z() / qn
                    };
                }
            }
        }
    }

    #if APSA_USE_INNOVATION_MON
    {
        std::array<scalar, kImuAxisCount> innov;
        std::array<scalar, kImuAxisCount> S_diag;

        innov[0] = gx.get();
        innov[1] = gy.get();
        innov[2] = gz.get();
        innov[3] = ax.get();
        innov[4] = ay.get();
        innov[5] = az.get();

        for (std::size_t i = 0U; i < kImuAxisCount; ++i) {
            S_diag[i] = 1e-2f;
        }

        if (imu_axis_nis_calculate(innov, S_diag, g_nis)) {
            (void)imu_health_all_axes(g_nis.nis_values, kImuHealthMaxAxes, g_health);
            (void)imu_health_smooth(g_health.overall_health, g_smoother);
        } else {
            agent_log_event(AGENT_LOG_EKF_DIVERGE, 0xE010U, g_status.update_count);
        }
    }
    #endif

    if (!eskf_inject_reset(g_eskf.delta_x, g_nominal, g_eskf)) {
        g_status.diverged = true;
        agent_log_event(AGENT_LOG_EKF_DIVERGE, 0xE003U, g_status.update_count);
        return AGENT_ERR_HW;
    }

    g_status.update_count++;
    g_status.last_update_us = now;

    #if APSA_USE_INNOVATION_MON
    g_status.confidence = g_smoother.smoothed_health;
    #else
    g_status.confidence = 1.0f;
    #endif
#endif

    return AGENT_OK;
}

agent_status_t ekf_predict(float dt)
{
    (void)dt;
    return AGENT_OK;
}

agent_status_t ekf_get_state(agent_ekf_state_t *out)
{
    if (out == NULL)           { return AGENT_ERR; }
    if (!g_status.initialized) { return AGENT_ERR_NOT_INIT; }

#if APSA_USE_ESKF
    out->pos[0] = g_nominal.pos_x.get();
    out->pos[1] = g_nominal.pos_y.get();
    out->pos[2] = g_nominal.pos_z.get();

    out->vel[0] = g_nominal.vel_x.get();
    out->vel[1] = g_nominal.vel_y.get();
    out->vel[2] = g_nominal.vel_z.get();

    out->quat[0] = g_nominal.attitude.get_w();
    out->quat[1] = g_nominal.attitude.get_x();
    out->quat[2] = g_nominal.attitude.get_y();
    out->quat[3] = g_nominal.attitude.get_z();

    out->gyro_bias[0] = g_nominal.bias.gyro_bias_x.get();
    out->gyro_bias[1] = g_nominal.bias.gyro_bias_y.get();
    out->gyro_bias[2] = g_nominal.bias.gyro_bias_z.get();

    out->accel_bias[0] = g_nominal.bias.accel_bias_x.get();
    out->accel_bias[1] = g_nominal.bias.accel_bias_y.get();
    out->accel_bias[2] = g_nominal.bias.accel_bias_z.get();
#else
    memset(out, 0, sizeof(*out));
    out->quat[0] = 1.0f;
#endif

    out->confidence   = g_status.confidence;
    out->timestamp_us = agent_time_now_us();

    return AGENT_OK;
}

agent_status_t ekf_health(void)
{
    if (!g_status.initialized) { return AGENT_ERR_NOT_INIT; }
    if (g_status.diverged)     { return AGENT_ERR_HW; }
    return AGENT_OK;
}

void ekf_on_failure(void)
{
    g_status.diverged = true;
    agent_log_event(AGENT_LOG_EKF_DIVERGE, 0xE004U, g_status.update_count);
}

bool ekf_has_diverged(void)
{
    return g_status.diverged;
}

const apsa_bridge_ekf_status_t *apsa_bridge_ekf_status(void)
{
    return &g_status;
}