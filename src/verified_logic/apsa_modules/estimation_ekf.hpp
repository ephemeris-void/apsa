#pragma once

#include "apsa_estimation_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"
#include "estimation_types.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA ESTIMATION — EXTENDED KALMAN FILTER
// Nonlinear systems via linearization
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// EKF DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kEkfMaxState{21};
constexpr std::size_t kEkfMaxControl{6};
constexpr std::size_t kEkfMaxMeasurement{9};

// ═══════════════════════════════════════════
// FUNCTION POINTER TYPES
// ═══════════════════════════════════════════

using StateTransitionFn = StateTransitionFnT<kEkfMaxState, kEkfMaxControl>;
using MeasurementFn = MeasurementFnT<kEkfMaxState, kEkfMaxMeasurement>;
using StateJacobianFn = StateJacobianFnT<kEkfMaxState, kEkfMaxControl>;
using MeasurementJacobianFn = MeasurementJacobianFnT<kEkfMaxState, kEkfMaxMeasurement>;

// ═══════════════════════════════════════════
// EKF CONFIG
// ═══════════════════════════════════════════

struct EkfConfig {
    std::size_t state_dim{0U};
    std::size_t control_dim{0U};
    std::size_t measurement_dim{0U};

    StateTransitionFn f{nullptr};
    MeasurementFn h{nullptr};
    StateJacobianFn F_jacobian{nullptr};
    MeasurementJacobianFn H_jacobian{nullptr};
};

// ═══════════════════════════════════════════
// EKF STATE
// ═══════════════════════════════════════════

struct EkfState {
    Vector_Dynamic<kEkfMaxState> x;
    Matrix_Dynamic<kEkfMaxState, kEkfMaxState> P;
    bool initialized{false};
    
    explicit EkfState(std::size_t n = kEkfMaxState) noexcept
        : x(n), P(n, n) {}
};

// ═══════════════════════════════════════════
// EKF WORKSPACE
// ═══════════════════════════════════════════

struct EkfWorkspace {
    Vector_Dynamic<kEkfMaxState> x_pred;
    Matrix_Dynamic<kEkfMaxState, kEkfMaxState> P_pred;
    Vector_Dynamic<kEkfMaxMeasurement> innovation;
    Vector_Dynamic<kEkfMaxMeasurement> z_pred;
    Matrix_Dynamic<kEkfMaxMeasurement, kEkfMaxMeasurement> S;
    Matrix_Dynamic<kEkfMaxState, kEkfMaxMeasurement> K;
    Matrix_Dynamic<kEkfMaxMeasurement, kEkfMaxState> PH_T;
    Matrix_Dynamic<kEkfMaxState, kEkfMaxState> I_minus_KH;
    
    explicit EkfWorkspace(
        std::size_t n = kEkfMaxState,
        std::size_t m = kEkfMaxMeasurement
    ) noexcept
        : x_pred(n)
        , P_pred(n, n)
        , innovation(m)
        , z_pred(m)
        , S(m, m)
        , K(n, m)
        , PH_T(m, n)
        , I_minus_KH(n, n)
    {}
};

// ═══════════════════════════════════════════
// INITIALIZE
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool ekf_initialize(
    const Vector_Dynamic<kEkfMaxState>& x0,
    const Matrix_Dynamic<kEkfMaxState, kEkfMaxState>& P0,
    EkfState& ekf
) noexcept {
    if (x0.size() == 0U || x0.size() > kEkfMaxState) APSA_UNLIKELY {
        return false;
    }

    if (P0.rows() != x0.size() || P0.cols() != x0.size()) APSA_UNLIKELY {
        return false;
    }

    if (!is_symmetric(P0)) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(P0)) APSA_UNLIKELY {
        return false;
    }

    ekf.x = x0;
    ekf.P = P0;
    ekf.initialized = true;

    return true;
}

// ═══════════════════════════════════════════
// PREDICT
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool ekf_predict(
    const EkfConfig& cfg,
    const Vector_Dynamic<kEkfMaxControl>& u,
    const Matrix_Dynamic<kEkfMaxState, kEkfMaxState>& Q,
    EkfState& ekf,
    EkfWorkspace& ws
) noexcept {
    if (!ekf.initialized) APSA_UNLIKELY {
        return false;
    }

    if (cfg.f == nullptr || cfg.F_jacobian == nullptr) APSA_UNLIKELY {
        return false;
    }

    const std::size_t n = ekf.x.size();

    if (Q.rows() != n || Q.cols() != n) APSA_UNLIKELY {
        return false;
    }

    ws.x_pred = cfg.f(ekf.x, u);

    if (!std::isfinite(ws.x_pred.at(0))) APSA_UNLIKELY {
        return false;
    }

    const auto F = cfg.F_jacobian(ekf.x, u);

    if (F.rows() != n || F.cols() != n) APSA_UNLIKELY {
        return false;
    }

    const auto FT = transpose(F);
    const auto FP = F * ekf.P;
    ws.P_pred = FP * FT + Q;

    ws.P_pred = (ws.P_pred + transpose(ws.P_pred)) * scalar{0.5f};

    ekf.x = ws.x_pred;
    ekf.P = ws.P_pred;

    return true;
}

// ═══════════════════════════════════════════
// UPDATE — Cholesky solve
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool ekf_update(
    const EkfConfig& cfg,
    const Vector_Dynamic<kEkfMaxMeasurement>& z,
    const Matrix_Dynamic<kEkfMaxMeasurement, kEkfMaxMeasurement>& R,
    EkfState& ekf,
    EkfWorkspace& ws
) noexcept {
    if (!ekf.initialized) APSA_UNLIKELY {
        return false;
    }

    if (cfg.h == nullptr || cfg.H_jacobian == nullptr) APSA_UNLIKELY {
        return false;
    }

    const std::size_t n = ekf.x.size();
    const std::size_t m = z.size();

    if (R.rows() != m || R.cols() != m) APSA_UNLIKELY {
        return false;
    }

    ws.z_pred = cfg.h(ekf.x);

    ws.innovation = z - ws.z_pred;

    const auto H = cfg.H_jacobian(ekf.x);

    if (H.rows() != m || H.cols() != n) APSA_UNLIKELY {
        return false;
    }

    const auto HT = transpose(H);
    ws.S = H * ekf.P * HT + R;
    ws.S = (ws.S + transpose(ws.S)) * scalar{0.5f};

    ws.PH_T = ekf.P * HT;

    Matrix_Dynamic<kEkfMaxMeasurement, kEkfMaxMeasurement> L{m, m};

    if (!cholesky(ws.S, L)) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kEkfMaxMeasurement, kEkfMaxState> X{m, n};

    for (std::size_t col{0U}; col < n; ++col) {
        Vector_Dynamic<kEkfMaxMeasurement> b_col{m};

        for (std::size_t i{0U}; i < m; ++i) {
            b_col.at(i) = ws.PH_T.at(col, i);
        }

        Vector_Dynamic<kEkfMaxMeasurement> y_col{m};

        if (!forward_substitute(b_col, L, y_col)) APSA_UNLIKELY {
            return false;
        }

        const auto LT = transpose(L);

        Vector_Dynamic<kEkfMaxMeasurement> x_col{m};

        if (!backward_substitute(y_col, LT, x_col)) APSA_UNLIKELY {
            return false;
        }

        for (std::size_t i{0U}; i < m; ++i) {
            X.at(i, col) = x_col.at(i);
        }
    }

    for (std::size_t i{0U}; i < n; ++i) {
        for (std::size_t j{0U}; j < m; ++j) {
            ws.K.at(i, j) = X.at(j, i);
        }
    }

    const auto Knu = ws.K * ws.innovation;
    ekf.x = ekf.x + Knu;

    for (std::size_t i{0U}; i < n; ++i) {
        for (std::size_t j{0U}; j < n; ++j) {
            scalar kh{0.0f};

            for (std::size_t k{0U}; k < m; ++k) {
                kh += ws.K.at(i, k) * H.at(k, j);
            }

            ws.I_minus_KH.at(i, j) =
                ((i == j) ? scalar{1.0f} : scalar{0.0f}) - kh;
        }
    }

    const auto I_minus_KH_T = transpose(ws.I_minus_KH);
    const auto KR = ws.K * R;
    const auto KRKT = KR * transpose(ws.K);

    ekf.P = ws.I_minus_KH * ekf.P * I_minus_KH_T + KRKT;
    ekf.P = (ekf.P + transpose(ekf.P)) * scalar{0.5f};

    if (!is_positive_definite(ekf.P)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// FULL STEP
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool ekf_step(
    const EkfConfig& cfg,
    const Vector_Dynamic<kEkfMaxControl>& u,
    const Vector_Dynamic<kEkfMaxMeasurement>& z,
    const Matrix_Dynamic<kEkfMaxState, kEkfMaxState>& Q,
    const Matrix_Dynamic<kEkfMaxMeasurement, kEkfMaxMeasurement>& R,
    EkfState& ekf,
    EkfWorkspace& ws
) noexcept {
    if (!ekf_predict(cfg, u, Q, ekf, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!ekf_update(cfg, z, R, ekf, ws)) APSA_UNLIKELY {
        return false;
    }

    return true;
}