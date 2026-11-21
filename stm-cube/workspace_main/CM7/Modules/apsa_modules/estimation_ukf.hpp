#pragma once

#include "apsa_estimation_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"
#include "estimation_types.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA ESTIMATION — UNSCENTED KALMAN FILTER
// Sigma-point propagation for nonlinear systems
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// UKF DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kUkfMaxState{21};
constexpr std::size_t kUkfMaxMeasurement{9};
constexpr std::size_t kUkfMaxSigma{43};
constexpr std::size_t kUkfMaxControl{6};

// ═══════════════════════════════════════════
// UKF PARAMETERS
// ═══════════════════════════════════════════

constexpr scalar kUkfAlpha{1e-3};
constexpr scalar kUkfBeta{2.0f};
constexpr scalar kUkfKappa{0.0f};
constexpr scalar kUkfSqrtThreshold{1e-12};

// ═══════════════════════════════════════════
// UKF CONFIG
// ═══════════════════════════════════════════

using UkfStateTransitionFn = StateTransitionFnT<kUkfMaxState, kUkfMaxControl>;
using UkfMeasurementFn = MeasurementFnT<kUkfMaxState, kUkfMaxMeasurement>;

struct UkfConfig {
    std::size_t state_dim{0U};
    std::size_t measurement_dim{0U};

    UkfStateTransitionFn f{nullptr};
    UkfMeasurementFn h{nullptr};
};

// ═══════════════════════════════════════════
// UKF STATE
// ═══════════════════════════════════════════

struct UkfState {
    Vector_Dynamic<kUkfMaxState> x;
    Matrix_Dynamic<kUkfMaxState, kUkfMaxState> P;
    bool initialized{false};
    
    explicit UkfState(std::size_t n = kUkfMaxState) noexcept
        : x(n), P(n, n) {}
};

// ═══════════════════════════════════════════
// UKF WORKSPACE
// ═══════════════════════════════════════════

struct UkfWorkspace {
    Matrix_Dynamic<kUkfMaxSigma, kUkfMaxState> sigma_points;
    Matrix_Dynamic<kUkfMaxSigma, kUkfMaxState> sigma_pred;
    Matrix_Dynamic<kUkfMaxSigma, kUkfMaxMeasurement> zeta;
    Vector_Dynamic<kUkfMaxSigma> weight_mean;
    Vector_Dynamic<kUkfMaxSigma> weight_cov;
    Vector_Dynamic<kUkfMaxState> x_pred;
    Matrix_Dynamic<kUkfMaxState, kUkfMaxState> P_pred;
    Vector_Dynamic<kUkfMaxMeasurement> z_pred;
    Matrix_Dynamic<kUkfMaxMeasurement, kUkfMaxMeasurement> S;
    Matrix_Dynamic<kUkfMaxState, kUkfMaxMeasurement> P_xz;
    Matrix_Dynamic<kUkfMaxState, kUkfMaxMeasurement> K;
    Vector_Dynamic<kUkfMaxMeasurement> innovation;
    Matrix_Dynamic<kUkfMaxState, kUkfMaxState> L_state;
    Matrix_Dynamic<kUkfMaxMeasurement, kUkfMaxMeasurement> L_meas;
    
    explicit UkfWorkspace(
        std::size_t n = kUkfMaxState,
        std::size_t m = kUkfMaxMeasurement
    ) noexcept
        : sigma_points(2U * n + 1U, n)
        , sigma_pred(2U * n + 1U, n)
        , zeta(2U * n + 1U, m)
        , weight_mean(2U * n + 1U)
        , weight_cov(2U * n + 1U)
        , x_pred(n)
        , P_pred(n, n)
        , z_pred(m)
        , S(m, m)
        , P_xz(n, m)
        , K(n, m)
        , innovation(m)
        , L_state(n, n)
        , L_meas(m, m)
    {}
};

// ═══════════════════════════════════════════
// INITIALIZE
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_initialize(
    const Vector_Dynamic<kUkfMaxState>& x0,
    const Matrix_Dynamic<kUkfMaxState, kUkfMaxState>& P0,
    UkfState& ukf
) noexcept {
    if (x0.size() == 0U || x0.size() > kUkfMaxState) APSA_UNLIKELY {
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

    ukf.x = x0;
    ukf.P = P0;
    ukf.initialized = true;

    return true;
}

// ═══════════════════════════════════════════
// WEIGHTS CALCULATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_calculate_weights(
    const std::size_t n,
    const scalar lambda,
    UkfWorkspace& ws
) noexcept {
    const scalar denom = static_cast<scalar>(n) + lambda;

    if (std::abs(denom) < kUkfSqrtThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar inv_denom = scalar{1.0f} / denom;

    ws.weight_mean.at(0) = lambda * inv_denom;
    ws.weight_cov.at(0) = lambda * inv_denom +
        (scalar{1.0f} - kUkfAlpha * kUkfAlpha + kUkfBeta);

    const scalar w_i = inv_denom * scalar{0.5f};

    for (std::size_t i{1U}; i < 2U * n + 1U; ++i) {
        ws.weight_mean.at(i) = w_i;
        ws.weight_cov.at(i) = w_i;
    }

    return true;
}

// ═══════════════════════════════════════════
// LAMBDA CALCULATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_calculate_lambda(
    const std::size_t n,
    scalar& lambda
) noexcept {
    lambda = kUkfAlpha * kUkfAlpha *
        (static_cast<scalar>(n) + kUkfKappa) - static_cast<scalar>(n);

    return std::isfinite(lambda);
}

// ═══════════════════════════════════════════
// SIGMA POINT GENERATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_generate_sigma_points(
    const UkfState& ukf,
    const scalar lambda,
    UkfWorkspace& ws
) noexcept {
    const std::size_t n = ukf.x.size();

    if (!is_positive_definite(ukf.P)) APSA_UNLIKELY {
        return false;
    }

    if (!cholesky(ukf.P, ws.L_state)) APSA_UNLIKELY {
        return false;
    }

    const scalar sqrt_factor = std::sqrt(
        static_cast<scalar>(n) + lambda
    );

    if (!std::isfinite(sqrt_factor)) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t j{0U}; j < n; ++j) {
        ws.sigma_points.at(0, j) = ukf.x.at(j);
    }

    for (std::size_t i{0U}; i < n; ++i) {
        for (std::size_t j{0U}; j < n; ++j) {
            const scalar diff = sqrt_factor * ws.L_state.at(j, i);

            ws.sigma_points.at(i + 1U, j) = ukf.x.at(j) + diff;
            ws.sigma_points.at(i + n + 1U, j) = ukf.x.at(j) - diff;
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// UKF PREDICT
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_predict(
    const UkfConfig& cfg,
    const Matrix_Dynamic<kUkfMaxState, kUkfMaxState>& Q,
    UkfState& ukf,
    UkfWorkspace& ws
) noexcept {
    if (!ukf.initialized) APSA_UNLIKELY {
        return false;
    }

    if (cfg.f == nullptr) APSA_UNLIKELY {
        return false;
    }

    const std::size_t n = ukf.x.size();

    scalar lambda{0.0f};

    if (!ukf_calculate_lambda(n, lambda)) APSA_UNLIKELY {
        return false;
    }

    if (!ukf_calculate_weights(n, lambda, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!ukf_generate_sigma_points(ukf, lambda, ws)) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        Vector_Dynamic<kUkfMaxState> x_i{n};

        for (std::size_t j{0U}; j < n; ++j) {
            x_i.at(j) = ws.sigma_points.at(i, j);
        }

        const auto f_result = cfg.f(x_i, Vector_Dynamic<kUkfMaxControl>{});

        for (std::size_t j{0U}; j < n; ++j) {
            ws.sigma_pred.at(i, j) = f_result.at(j);
        }
    }

    ws.x_pred = Vector_Dynamic<kUkfMaxState>{n};

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        for (std::size_t j{0U}; j < n; ++j) {
            ws.x_pred.at(j) += ws.weight_mean.at(i) * ws.sigma_pred.at(i, j);
        }
    }

    ws.P_pred = Matrix_Dynamic<kUkfMaxState, kUkfMaxState>{n, n};

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        for (std::size_t a{0U}; a < n; ++a) {
            const scalar da = ws.sigma_pred.at(i, a) - ws.x_pred.at(a);

            for (std::size_t b{0U}; b < n; ++b) {
                const scalar db = ws.sigma_pred.at(i, b) - ws.x_pred.at(b);
                ws.P_pred.at(a, b) += ws.weight_cov.at(i) * da * db;
            }
        }
    }

    ws.P_pred = ws.P_pred + Q;
    ws.P_pred = (ws.P_pred + transpose(ws.P_pred)) * scalar{0.5f};

    ukf.x = ws.x_pred;
    ukf.P = ws.P_pred;

    return true;
}

// ═══════════════════════════════════════════
// UKF UPDATE — Cholesky Solve
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_update(
    const UkfConfig& cfg,
    const Vector_Dynamic<kUkfMaxMeasurement>& z,
    const Matrix_Dynamic<kUkfMaxMeasurement, kUkfMaxMeasurement>& R,
    UkfState& ukf,
    UkfWorkspace& ws
) noexcept {
    if (!ukf.initialized) APSA_UNLIKELY {
        return false;
    }

    if (cfg.h == nullptr) APSA_UNLIKELY {
        return false;
    }

    const std::size_t n = ukf.x.size();
    const std::size_t m = z.size();

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        Vector_Dynamic<kUkfMaxState> x_i{n};

        for (std::size_t j{0U}; j < n; ++j) {
            x_i.at(j) = ws.sigma_pred.at(i, j);
        }

        const auto h_result = cfg.h(x_i);

        for (std::size_t j{0U}; j < m; ++j) {
            ws.zeta.at(i, j) = h_result.at(j);
        }
    }

    ws.z_pred = Vector_Dynamic<kUkfMaxMeasurement>{m};

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        for (std::size_t j{0U}; j < m; ++j) {
            ws.z_pred.at(j) += ws.weight_mean.at(i) * ws.zeta.at(i, j);
        }
    }

    ws.S = Matrix_Dynamic<kUkfMaxMeasurement, kUkfMaxMeasurement>{m, m};

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        for (std::size_t a{0U}; a < m; ++a) {
            const scalar da = ws.zeta.at(i, a) - ws.z_pred.at(a);

            for (std::size_t b{0U}; b < m; ++b) {
                const scalar db = ws.zeta.at(i, b) - ws.z_pred.at(b);
                ws.S.at(a, b) += ws.weight_cov.at(i) * da * db;
            }
        }
    }

    ws.S = ws.S + R;
    ws.S = (ws.S + transpose(ws.S)) * scalar{0.5f};

    ws.P_xz = Matrix_Dynamic<kUkfMaxState, kUkfMaxMeasurement>{n, m};

    for (std::size_t i{0U}; i < 2U * n + 1U; ++i) {
        for (std::size_t a{0U}; a < n; ++a) {
            const scalar da = ws.sigma_pred.at(i, a) - ws.x_pred.at(a);

            for (std::size_t b{0U}; b < m; ++b) {
                const scalar db = ws.zeta.at(i, b) - ws.z_pred.at(b);
                ws.P_xz.at(a, b) += ws.weight_cov.at(i) * da * db;
            }
        }
    }

    if (!cholesky(ws.S, ws.L_meas)) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kUkfMaxMeasurement, kUkfMaxState> X{m, n};

    for (std::size_t col{0U}; col < n; ++col) {
        Vector_Dynamic<kUkfMaxMeasurement> b_col{m};

        for (std::size_t i{0U}; i < m; ++i) {
            b_col.at(i) = ws.P_xz.at(col, i);
        }

        Vector_Dynamic<kUkfMaxMeasurement> y_col{m};

        if (!forward_substitute(b_col, ws.L_meas, y_col)) APSA_UNLIKELY {
            return false;
        }

        const auto L_meas_T = transpose(ws.L_meas);

        Vector_Dynamic<kUkfMaxMeasurement> x_col{m};

        if (!backward_substitute(y_col, L_meas_T, x_col)) APSA_UNLIKELY {
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

    ws.innovation = z - ws.z_pred;

    const auto Knu = ws.K * ws.innovation;
    ukf.x = ukf.x + Knu;

    const auto KSKT = ws.K * ws.S * transpose(ws.K);
    ukf.P = ukf.P - KSKT;

    ukf.P = (ukf.P + transpose(ukf.P)) * scalar{0.5f};

    if (!is_positive_definite(ukf.P)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// FULL UKF STEP
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool ukf_step(
    const UkfConfig& cfg,
    const Vector_Dynamic<kUkfMaxMeasurement>& z,
    const Matrix_Dynamic<kUkfMaxState, kUkfMaxState>& Q,
    const Matrix_Dynamic<kUkfMaxMeasurement, kUkfMaxMeasurement>& R,
    UkfState& ukf,
    UkfWorkspace& ws
) noexcept {
    if (!ukf_predict(cfg, Q, ukf, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!ukf_update(cfg, z, R, ukf, ws)) APSA_UNLIKELY {
        return false;
    }

    return true;
}