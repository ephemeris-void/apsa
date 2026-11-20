#pragma once

#include "apsa_estimation_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

// ═══════════════════════════════════════════
// APSA ESTIMATION — ERROR STATE KALMAN FILTER
// 19-state with IMU bias augmentation
// Universal — noise parameter-driven
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// ESKF DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kEskfStateDim{19};
constexpr std::size_t kEskfErrorStateDim{18};
constexpr std::size_t kEskfControlDim{6};
constexpr std::size_t kEskfMeasurementDim{9};

// ═══════════════════════════════════════════
// ESKF PROCESS NOISE CONFIG
// ═══════════════════════════════════════════

struct EskfProcessNoiseConfig {
    scalar gyro_arw{0.0f};
    scalar gyro_rrw{0.0f};
    scalar accel_vrw{0.0f};
    scalar accel_rrw{0.0f};
    scalar pos_rw{0.0f};
    scalar vel_rw{0.0f};
};

// ═══════════════════════════════════════════
// ESKF MEASUREMENT NOISE CONFIG
// ═══════════════════════════════════════════

struct EskfMeasurementNoiseConfig {
    scalar gps_pos{0.0f};
    scalar gps_vel{0.0f};
    scalar attitude{0.0f};
};

// ═══════════════════════════════════════════
// IMU BIAS STATE
// ═══════════════════════════════════════════

struct EskfImuBias {
    AngularVel gyro_bias_x{0.0f};
    AngularVel gyro_bias_y{0.0f};
    AngularVel gyro_bias_z{0.0f};
    Acceleration accel_bias_x{0.0f};
    Acceleration accel_bias_y{0.0f};
    Acceleration accel_bias_z{0.0f};
};

// ═══════════════════════════════════════════
// ESKF NOMINAL STATE
// ═══════════════════════════════════════════

struct EskfNominalState {
    Length pos_x{0.0f};
    Length pos_y{0.0f};
    Length pos_z{0.0f};
    Velocity vel_x{0.0f};
    Velocity vel_y{0.0f};
    Velocity vel_z{0.0f};
    Quaternion attitude{};
    EskfImuBias bias{};
};

// ═══════════════════════════════════════════
// ESKF ERROR STATE
// ═══════════════════════════════════════════

struct EskfErrorState {
    Vector_Dynamic<kEskfStateDim> delta_x;
    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> P;
    bool initialized{false};
    
    explicit EskfErrorState(std::size_t n = kEskfStateDim) noexcept
        : delta_x(n), P(n, n) {}
};

// ═══════════════════════════════════════════
// ESKF WORKSPACE
// ═══════════════════════════════════════════

struct EskfWorkspace {
    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> F;
    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> Phi;
    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> Q_d;
    Matrix_Dynamic<kEskfMeasurementDim, kEskfStateDim> H;
    Matrix_Dynamic<kEskfMeasurementDim, kEskfMeasurementDim> R;
    Matrix_Dynamic<kEskfStateDim, kEskfMeasurementDim> K;
    Vector_Dynamic<kEskfMeasurementDim> innovation;
    Matrix_Dynamic<kEskfMeasurementDim, kEskfMeasurementDim> S;
    Matrix_Dynamic<kEskfMeasurementDim, kEskfStateDim> PH_T;
    
    explicit EskfWorkspace(
        std::size_t n = kEskfStateDim,
        std::size_t m = kEskfMeasurementDim
    ) noexcept
        : F(n, n)
        , Phi(n, n)
        , Q_d(n, n)
        , H(m, n)
        , R(m, m)
        , K(n, m)
        , innovation(m)
        , S(m, m)
        , PH_T(m, n)
    {}
};

// ═══════════════════════════════════════════
// SKEW-SYMMETRIC MATRIX
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr Mat3 eskf_skew_symmetric(
    const scalar vx,
    const scalar vy,
    const scalar vz
) noexcept {
    return Mat3{{
        0.0f, -vz,  vy,
        vz,   0.0f, -vx,
       -vy,   vx,   0.0f
    }};
}

// ═══════════════════════════════════════════
// ESKF INITIALIZE
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_initialize(
    const EskfNominalState& nominal,
    const Matrix_Dynamic<kEskfStateDim, kEskfStateDim>& P0,
    EskfErrorState& eskf
) noexcept {
    const scalar q_norm_sq = norm_sq(nominal.attitude);

    if (std::abs(q_norm_sq - scalar{1.0f}) > scalar{1e-6}) APSA_UNLIKELY {
        return false;
    }

    if (!is_symmetric(P0)) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(P0)) APSA_UNLIKELY {
        return false;
    }

    eskf.delta_x = Vector_Dynamic<kEskfStateDim>{kEskfStateDim};
    eskf.P = P0;
    eskf.initialized = true;

    return true;
}

// ═══════════════════════════════════════════
// BUILD F MATRIX
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_build_F(
    const EskfNominalState& nominal,
    const AngularVel omega_x,
    const AngularVel omega_y,
    const AngularVel omega_z,
    const Acceleration accel_x,
    const Acceleration accel_y,
    const Acceleration accel_z,
    EskfWorkspace& ws
) noexcept {
    const scalar wx = omega_x.get();
    const scalar wy = omega_y.get();
    const scalar wz = omega_z.get();

    const scalar ax = accel_x.get();
    const scalar ay = accel_y.get();
    const scalar az = accel_z.get();

    const Mat3 omega_skew = eskf_skew_symmetric(wx, wy, wz);
    const Mat3 accel_skew = eskf_skew_symmetric(ax, ay, az);

    const Quaternion q = nominal.attitude;
    const Mat3 R = rotation_matrix_from_quaternion(q);

    const Mat3 R_accel_skew = R * accel_skew;

    ws.F = Matrix_Dynamic<kEskfStateDim, kEskfStateDim>{
        kEskfStateDim, kEskfStateDim
    };

    for (std::size_t i{0U}; i < 3U; ++i) {
        for (std::size_t j{0U}; j < 3U; ++j) {
            ws.F.at(i, j + 3U) = (i == j) ? scalar{1.0f} : scalar{0.0f};

            ws.F.at(i + 3U, j + 6U) = -R_accel_skew.at(i, j);

            ws.F.at(i + 3U, j + 12U) = -R.at(i, j);

            ws.F.at(i + 6U, j + 6U) = -omega_skew.at(i, j);

            ws.F.at(i + 6U, j + 9U) = (i == j) ? scalar{-1.0f} : scalar{0.0f};
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// BUILD Q MATRIX — Noise Config Driven
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_build_Q(
    const Time dt,
    const EskfProcessNoiseConfig& noise,
    EskfWorkspace& ws
) noexcept {
    if (dt.get() <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar dt_val = dt.get();

    const scalar q_gyro = noise.gyro_arw * noise.gyro_arw / dt_val;
    const scalar q_gyro_bias = noise.gyro_rrw * noise.gyro_rrw * dt_val;

    const scalar q_accel_bias = noise.accel_rrw * noise.accel_rrw * dt_val;

    const scalar q_pos = noise.pos_rw * noise.pos_rw * dt_val;
    const scalar q_vel = noise.vel_rw * noise.vel_rw * dt_val;

    ws.Q_d = Matrix_Dynamic<kEskfStateDim, kEskfStateDim>{
        kEskfStateDim, kEskfStateDim
    };

    for (std::size_t i{0U}; i < 3U; ++i) {
        ws.Q_d.at(i,      i)      = q_pos;
        ws.Q_d.at(i + 3U, i + 3U) = q_vel;
        ws.Q_d.at(i + 6U, i + 6U) = q_gyro;
        ws.Q_d.at(i + 9U, i + 9U) = q_gyro_bias;
        ws.Q_d.at(i + 12U, i + 12U) = q_accel_bias;
    }

    return true;
}

// ═══════════════════════════════════════════
// BUILD PHI — 1st order ZOH
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_build_Phi(
    const Time dt,
    EskfWorkspace& ws
) noexcept {
    const scalar dt_val = dt.get();

    if (dt_val <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> I{
        kEskfStateDim, kEskfStateDim
    };

    for (std::size_t i{0U}; i < kEskfStateDim; ++i) {
        I.at(i, i) = scalar{1.0f};
    }

    const auto Fdt = ws.F * dt_val;
    const auto Fdt2 = Fdt * Fdt * scalar{0.5f};

    ws.Phi = I + Fdt + Fdt2;

    return true;
}

// ═══════════════════════════════════════════
// BUILD H MATRIX — Position + Velocity + Attitude
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_build_H(
    const std::size_t measurement_dim,
    EskfWorkspace& ws
) noexcept {
    if (measurement_dim > kEskfMeasurementDim) APSA_UNLIKELY {
        return false;
    }

    ws.H = Matrix_Dynamic<kEskfMeasurementDim, kEskfStateDim>{
        measurement_dim, kEskfStateDim
    };

    /* Position state (0, 1, 2) */
    for (std::size_t i{0U}; i < 3U && i < measurement_dim; ++i) {
        ws.H.at(i, i) = scalar{1.0f};
    }

    /* Velocity state (3, 4, 5) */
    for (std::size_t i{0U}; i < 3U && (i + 3U) < measurement_dim; ++i) {
        ws.H.at(i + 3U, i + 3U) = scalar{1.0f};
    }

    /* Attitude state (6, 7, 8) */
    for (std::size_t i{0U}; i < 3U && (i + 6U) < measurement_dim; ++i) {
        ws.H.at(i + 6U, i + 6U) = scalar{1.0f};
    }

    return true;
}

// ═══════════════════════════════════════════
// BUILD R MATRIX — Noise Config Driven
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_build_R(
    const std::size_t measurement_dim,
    const EskfMeasurementNoiseConfig& noise,
    EskfWorkspace& ws
) noexcept {
    if (measurement_dim > kEskfMeasurementDim) APSA_UNLIKELY {
        return false;
    }

    ws.R = Matrix_Dynamic<kEskfMeasurementDim, kEskfMeasurementDim>{
        measurement_dim, measurement_dim
    };

    /* Position measurement noise */
    for (std::size_t i{0U}; i < 3U && i < measurement_dim; ++i) {
        ws.R.at(i, i) = noise.gps_pos * noise.gps_pos;
    }

    /* Velocity measurement noise */
    for (std::size_t i{3U}; i < 6U && i < measurement_dim; ++i) {
        ws.R.at(i, i) = noise.gps_vel * noise.gps_vel;
    }

    /* Attitude measurement noise (accelerometer-based) */
    for (std::size_t i{6U}; i < 9U && i < measurement_dim; ++i) {
        ws.R.at(i, i) = noise.attitude * noise.attitude;
    }

    return true;
}

// ═══════════════════════════════════════════
// ESKF PREDICT
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_predict(
    const EskfNominalState& nominal,
    const AngularVel omega_x,
    const AngularVel omega_y,
    const AngularVel omega_z,
    const Acceleration accel_x,
    const Acceleration accel_y,
    const Acceleration accel_z,
    const Time dt,
    const EskfProcessNoiseConfig& noise,
    EskfErrorState& eskf,
    EskfWorkspace& ws
) noexcept {
    if (!eskf.initialized) APSA_UNLIKELY {
        return false;
    }

    if (!eskf_build_F(nominal, omega_x, omega_y, omega_z,
                      accel_x, accel_y, accel_z, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!eskf_build_Q(dt, noise, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!eskf_build_Phi(dt, ws)) APSA_UNLIKELY {
        return false;
    }

    const auto PhiT = transpose(ws.Phi);
    const auto PhiP = ws.Phi * eskf.P;
    eskf.P = PhiP * PhiT + ws.Q_d;

    eskf.P = (eskf.P + transpose(eskf.P)) * scalar{0.5f};

    if (!is_positive_definite(eskf.P)) APSA_UNLIKELY {
        return false;
    }

    eskf.delta_x = Vector_Dynamic<kEskfStateDim>{kEskfStateDim};

    return true;
}

// ═══════════════════════════════════════════
// ESKF UPDATE — Cholesky Solve
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_update(
    const Vector_Dynamic<kEskfMeasurementDim>& z,
    const std::size_t measurement_dim,
    const EskfMeasurementNoiseConfig& noise,
    EskfErrorState& eskf,
    EskfWorkspace& ws
) noexcept {
    if (!eskf.initialized) APSA_UNLIKELY {
        return false;
    }

    // Resize measurement-dependent workspace to runtime dimension
    ws.H = Matrix_Dynamic<kEskfMeasurementDim, kEskfStateDim>{
        measurement_dim, kEskfStateDim
    };
    ws.R = Matrix_Dynamic<kEskfMeasurementDim, kEskfMeasurementDim>{
        measurement_dim, measurement_dim
    };
    ws.K = Matrix_Dynamic<kEskfStateDim, kEskfMeasurementDim>{
        kEskfStateDim, measurement_dim
    };
    ws.innovation = Vector_Dynamic<kEskfMeasurementDim>{measurement_dim};
    ws.S = Matrix_Dynamic<kEskfMeasurementDim, kEskfMeasurementDim>{
        measurement_dim, measurement_dim
    };
    ws.PH_T = Matrix_Dynamic<kEskfMeasurementDim, kEskfStateDim>{
        measurement_dim, kEskfStateDim
    };

    if (!eskf_build_H(measurement_dim, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!eskf_build_R(measurement_dim, noise, ws)) APSA_UNLIKELY {
        return false;
    }

    // Copy only measurement_dim elements
    for (std::size_t i{0U}; i < measurement_dim; ++i) {
        ws.innovation.at(i) = z.at(i);
    }

    const auto HT = transpose(ws.H);
    ws.S = ws.H * eskf.P * HT + ws.R;
    ws.S = (ws.S + transpose(ws.S)) * scalar{0.5f};

    ws.PH_T = eskf.P * HT;
    
    Matrix_Dynamic<kEskfMeasurementDim, kEskfMeasurementDim> L{
        measurement_dim, measurement_dim
    };

    if (!cholesky(ws.S, L)) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kEskfMeasurementDim, kEskfStateDim> X{
        measurement_dim, kEskfStateDim
    };

    for (std::size_t col{0U}; col < kEskfStateDim; ++col) {
        Vector_Dynamic<kEskfMeasurementDim> b_col{measurement_dim};

        for (std::size_t i{0U}; i < measurement_dim; ++i) {
            b_col.at(i) = ws.PH_T.at(col, i);
        }

        Vector_Dynamic<kEskfMeasurementDim> y_col{measurement_dim};

        if (!forward_substitute(b_col, L, y_col)) APSA_UNLIKELY {
            return false;
        }

        const auto LT = transpose(L);

        Vector_Dynamic<kEskfMeasurementDim> x_col{measurement_dim};

        if (!backward_substitute(y_col, LT, x_col)) APSA_UNLIKELY {
            return false;
        }

        for (std::size_t i{0U}; i < measurement_dim; ++i) {
            X.at(i, col) = x_col.at(i);
        }
    }

    for (std::size_t i{0U}; i < kEskfStateDim; ++i) {
        for (std::size_t j{0U}; j < measurement_dim; ++j) {
            ws.K.at(i, j) = X.at(j, i);
        }
    }

    const auto Knu = ws.K * ws.innovation;
    eskf.delta_x = eskf.delta_x + Knu;

    Matrix_Dynamic<kEskfStateDim, kEskfStateDim> I_minus_KH{
        kEskfStateDim, kEskfStateDim
    };

    for (std::size_t i{0U}; i < kEskfStateDim; ++i) {
        for (std::size_t j{0U}; j < kEskfStateDim; ++j) {
            scalar kh{0.0f};

            for (std::size_t k{0U}; k < measurement_dim; ++k) {
                kh += ws.K.at(i, k) * ws.H.at(k, j);
            }

            I_minus_KH.at(i, j) =
                ((i == j) ? scalar{1.0f} : scalar{0.0f}) - kh;
        }
    }

    const auto I_minus_KH_T = transpose(I_minus_KH);
    const auto KR = ws.K * ws.R;
    const auto KRKT = KR * transpose(ws.K);

    eskf.P = I_minus_KH * eskf.P * I_minus_KH_T + KRKT;
    eskf.P = (eskf.P + transpose(eskf.P)) * scalar{0.5f};

    if (!is_positive_definite(eskf.P)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// ESKF INJECT + RESET
// ═══════════════════════════════════════════

[[nodiscard]] inline bool eskf_inject_reset(
    const Vector_Dynamic<kEskfStateDim>& delta_x,
    EskfNominalState& nominal,
    EskfErrorState& eskf
) noexcept {
    nominal.pos_x = Length{nominal.pos_x.get() + delta_x.at(0)};
    nominal.pos_y = Length{nominal.pos_y.get() + delta_x.at(1)};
    nominal.pos_z = Length{nominal.pos_z.get() + delta_x.at(2)};

    nominal.vel_x = Velocity{nominal.vel_x.get() + delta_x.at(3)};
    nominal.vel_y = Velocity{nominal.vel_y.get() + delta_x.at(4)};
    nominal.vel_z = Velocity{nominal.vel_z.get() + delta_x.at(5)};

    const scalar dqx = delta_x.at(6) * scalar{0.5f};
    const scalar dqy = delta_x.at(7) * scalar{0.5f};
    const scalar dqz = delta_x.at(8) * scalar{0.5f};

    const Quaternion delta_q{
        scalar{1.0f}, dqx, dqy, dqz
    };

    Quaternion delta_q_norm{};
    if (!quaternion_normalize(delta_q, delta_q_norm)) APSA_UNLIKELY {
        return false;
    }

    nominal.attitude = nominal.attitude * delta_q_norm;

    if (!quaternion_normalize(nominal.attitude, nominal.attitude)) {
        APSA_UNLIKELY { return false; }
    }

    nominal.bias.gyro_bias_x = AngularVel{
        nominal.bias.gyro_bias_x.get() + delta_x.at(9)
    };
    nominal.bias.gyro_bias_y = AngularVel{
        nominal.bias.gyro_bias_y.get() + delta_x.at(10)
    };
    nominal.bias.gyro_bias_z = AngularVel{
        nominal.bias.gyro_bias_z.get() + delta_x.at(11)
    };

    nominal.bias.accel_bias_x = Acceleration{
        nominal.bias.accel_bias_x.get() + delta_x.at(12)
    };
    nominal.bias.accel_bias_y = Acceleration{
        nominal.bias.accel_bias_y.get() + delta_x.at(13)
    };
    nominal.bias.accel_bias_z = Acceleration{
        nominal.bias.accel_bias_z.get() + delta_x.at(14)
    };

    eskf.delta_x = Vector_Dynamic<kEskfStateDim>{kEskfStateDim};

    return true;
}