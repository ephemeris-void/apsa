#include "apsa_bridge.h"
#include "apsa_config.h"
#include "../../apsa_modules/estimation_eskf.hpp"
#include "../../apsa_modules/estimation_innovation_monitor.hpp"
#include "../../agent/interfaces/agent_ekf_if.h"
#include "../../agent/interfaces/agent_imu_if.h"
#include "../../agent/core/agent_context.h"
#include "../../agent/core/agent_config.h"
#include "../../agent/trace/agent_log.h"
#include <string.h>
#include <cmath>

#if APSA_USE_ESKF
static EskfNominalState           g_nominal;
static EskfErrorState             g_eskf;
static EskfWorkspace              g_ws;
static EskfProcessNoiseConfig     g_proc_noise;
static EskfMeasurementNoiseConfig g_meas_noise;
#endif

#if APSA_USE_INNOVATION_MON
static ImuAxisNisState            g_nis;
static ImuHealthState             g_health;
static ImuHealthSmoother          g_smoother;
#endif

static apsa_bridge_ekf_status_t   g_status;

#define BRIDGE_MG_TO_MS2     (9.80665f / 2048.0f)
#define BRIDGE_DPS_TO_RADS   (0.0174532925f / 16.4f)

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
#endif

#if APSA_USE_INNOVATION_MON
    g_nis = ImuAxisNisState{};
    g_health = ImuHealthState{};
    g_smoother = ImuHealthSmoother{};
    g_smoother.alpha = 0.1f;
    g_smoother.initialized = false;
#endif

    g_status.update_count   = 0U;
    g_status.last_update_us = agent_time_now_us();
    g_status.confidence     = 0.0f;
    g_status.diverged       = false;
    g_status.initialized    = true;

    return AGENT_OK;
}

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

    g_status.update_count   = 0U;
    g_status.last_update_us = agent_time_now_us();
    g_status.diverged       = false;
    g_status.confidence     = 1.0f;
#endif
}

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

    if (imu_health() == AGENT_HEALTH_FAIL) {
        g_nominal.vel_x = Velocity{0.0f};
        g_nominal.vel_y = Velocity{0.0f};
        g_nominal.vel_z = Velocity{0.0f};
        g_status.update_count++;
        g_status.last_update_us = agent_time_now_us();
        return AGENT_OK;
    }

    uint32_t now   = agent_time_now_us();
    uint32_t dt_us = now - g_status.last_update_us;
    float    dt_s  = (float)dt_us * 1e-6f;

    if (dt_s < 1e-4f) { dt_s = 1e-4f; }
    if (dt_s > 5e-2f) { dt_s = 5e-2f; }

    AngularVel gx{BRIDGE_DPS_TO_RADS * (float)sample->gx};
    AngularVel gy{BRIDGE_DPS_TO_RADS * (float)sample->gy};
    AngularVel gz{BRIDGE_DPS_TO_RADS * (float)sample->gz};

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

    g_status.confidence = 1.0f;
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