#pragma once

#include "apsa_control_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA CONTROL — MODEL PREDICTIVE CONTROL
// MPC + Tube MPC
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// MPC DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kMpcMaxState{12};
constexpr std::size_t kMpcMaxControl{4};
constexpr std::size_t kMpcMaxHorizon{20};
constexpr std::size_t kMpcMaxInequality{40};

// ═══════════════════════════════════════════
// MPC CONVERGENCE CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kMpcMaxQpIterations{100};
constexpr scalar kMpcQpTolerance{1e-8};
constexpr scalar kMpcRegularization{1e-6};

// ═══════════════════════════════════════════
// TUBE MPC DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kTubeMpcMaxState{12};
constexpr std::size_t kTubeMpcMaxControl{4};
constexpr std::size_t kTubeMpcMaxHorizon{20};
constexpr std::size_t kTubeMpcMaxInequality{40};

// ═══════════════════════════════════════════
// TUBE MPC CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kTubeMpcMaxQpIterations{100};
constexpr scalar kTubeMpcQpTolerance{1e-8};
constexpr scalar kTubeMpcRegularization{1e-6};
constexpr scalar kTubeMpcAlpha{0.95f};
constexpr scalar kTubeMpcDisturbanceBound{0.1f};

// ═══════════════════════════════════════════
// MPC CONFIG
// ═══════════════════════════════════════════

struct MpcConfig {
    std::size_t state_dim{0U};
    std::size_t control_dim{0U};
    std::size_t horizon{0U};

    Matrix_Dynamic<kMpcMaxState, kMpcMaxState> Phi;
    Matrix_Dynamic<kMpcMaxState, kMpcMaxControl> Gamma;
    Matrix_Dynamic<kMpcMaxState, kMpcMaxState> Q;
    Matrix_Dynamic<kMpcMaxControl, kMpcMaxControl> R;
    Matrix_Dynamic<kMpcMaxState, kMpcMaxState> P_terminal;
    Vector_Dynamic<kMpcMaxControl> u_min;
    Vector_Dynamic<kMpcMaxControl> u_max;
    
    explicit MpcConfig(std::size_t n = kMpcMaxState,
                       std::size_t m = kMpcMaxControl) noexcept
        : Phi(n, n)
        , Gamma(n, m)
        , Q(n, n)
        , R(m, m)
        , P_terminal(n, n)
        , u_min(m)
        , u_max(m)
    {}
};

// ═══════════════════════════════════════════
// MPC WORKSPACE
// ═══════════════════════════════════════════

struct MpcWorkspace {
    Matrix_Dynamic<kMpcMaxState * kMpcMaxHorizon, kMpcMaxState> A_pred;
    Matrix_Dynamic<kMpcMaxState * kMpcMaxHorizon, kMpcMaxControl * kMpcMaxHorizon> B_pred;
    Matrix_Dynamic<kMpcMaxControl * kMpcMaxHorizon, kMpcMaxControl * kMpcMaxHorizon> H;
    Vector_Dynamic<kMpcMaxControl * kMpcMaxHorizon> f;
    Matrix_Dynamic<kMpcMaxInequality, kMpcMaxControl * kMpcMaxHorizon> A_ineq;
    Vector_Dynamic<kMpcMaxInequality> b_ineq;
    
    explicit MpcWorkspace(
        std::size_t n = kMpcMaxState,
        std::size_t m = kMpcMaxControl,
        std::size_t N = kMpcMaxHorizon
    ) noexcept
        : A_pred(n * N, n)
        , B_pred(n * N, m * N)
        , H(m * N, m * N)
        , f(m * N)
        , A_ineq(2 * m * N, m * N)
        , b_ineq(2 * m * N)
    {}
};

// ═══════════════════════════════════════════
// BUILD PREDICTION MATRICES
// ═══════════════════════════════════════════

[[nodiscard]] inline bool mpc_build_prediction(
    const MpcConfig& cfg,
    MpcWorkspace& ws
) noexcept {
    [[maybe_unused]] const std::size_t n = cfg.state_dim;
    const std::size_t m = cfg.control_dim;
    [[maybe_unused]] const std::size_t N = cfg.horizon;

    if (n == 0U || n > kMpcMaxState) APSA_UNLIKELY {
        return false;
    }

    if (m == 0U || m > kMpcMaxControl) APSA_UNLIKELY {
        return false;
    }

    if (N == 0U || N > kMpcMaxHorizon) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kMpcMaxState, kMpcMaxState> Phi_power = cfg.Phi;

    for (std::size_t i{0U}; i < N; ++i) {
        for (std::size_t a{0U}; a < n; ++a) {
            for (std::size_t b{0U}; b < n; ++b) {
                ws.A_pred.at(i * n + a, b) = Phi_power.at(a, b);
            }
        }

        Phi_power = Phi_power * cfg.Phi;
    }

    for (std::size_t i{0U}; i < N; ++i) {
        for (std::size_t j{0U}; j <= i; ++j) {
            Matrix_Dynamic<kMpcMaxState, kMpcMaxState> Phi_ij = [&]() {
                Matrix_Dynamic<kMpcMaxState, kMpcMaxState> temp{n, n};

                for (std::size_t a{0U}; a < n; ++a) {
                    temp.at(a, a) = scalar{1.0f};
                }

                for (std::size_t k{0U}; k < i - j; ++k) {
                    temp = temp * cfg.Phi;
                }

                return temp;
            }();

            const auto block = Phi_ij * cfg.Gamma;

            for (std::size_t a{0U}; a < n; ++a) {
                for (std::size_t b{0U}; b < m; ++b) {
                    ws.B_pred.at(i * n + a, j * m + b) = block.at(a, b);
                }
            }
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// BUILD QP COST
// ═══════════════════════════════════════════

[[nodiscard]] inline bool mpc_build_cost(
    const MpcConfig& cfg,
    const Vector_Dynamic<kMpcMaxState>& x_current,
    MpcWorkspace& ws
) noexcept {
    const std::size_t n = cfg.state_dim;
    const std::size_t m = cfg.control_dim;
    [[maybe_unused]] const std::size_t N = cfg.horizon;

    const std::size_t nu_total = m * N;

    Matrix_Dynamic<kMpcMaxState * kMpcMaxHorizon, kMpcMaxState * kMpcMaxHorizon> Q_bar{
        n * N, n * N
    };

    for (std::size_t i{0U}; i < N; ++i) {
        for (std::size_t a{0U}; a < n; ++a) {
            for (std::size_t b{0U}; b < n; ++b) {
                Q_bar.at(i * n + a, i * n + b) = cfg.Q.at(a, b);
            }
        }
    }

    for (std::size_t a{0U}; a < n; ++a) {
        for (std::size_t b{0U}; b < n; ++b) {
            Q_bar.at((N - 1U) * n + a, (N - 1U) * n + b) =
                cfg.P_terminal.at(a, b);
        }
    }

    Matrix_Dynamic<kMpcMaxControl * kMpcMaxHorizon, kMpcMaxControl * kMpcMaxHorizon> R_bar{
        nu_total, nu_total
    };

    for (std::size_t i{0U}; i < N; ++i) {
        for (std::size_t a{0U}; a < m; ++a) {
            for (std::size_t b{0U}; b < m; ++b) {
                R_bar.at(i * m + a, i * m + b) = cfg.R.at(a, b);
            }
        }
    }

    const auto BT = transpose(ws.B_pred);
    const auto BT_Q = BT * Q_bar;
    const auto BT_Q_B = BT_Q * ws.B_pred;

    ws.H = BT_Q_B + R_bar;
    ws.H = ws.H * scalar{2.0f};

    for (std::size_t i{0U}; i < nu_total; ++i) {
        ws.H.at(i, i) += kMpcRegularization;
    }

    const auto Ax = ws.A_pred * x_current;
    const auto Q_Ax = Q_bar * Ax;
    const auto BT_Q_Ax = BT * Q_Ax;

    for (std::size_t i{0U}; i < nu_total; ++i) {
        ws.f.at(i) = scalar{2.0f} * BT_Q_Ax.at(i);
    }

    return true;
}

// ═══════════════════════════════════════════
// BUILD INEQUALITY CONSTRAINTS
// ═══════════════════════════════════════════

[[nodiscard]] inline bool mpc_build_constraints(
    const MpcConfig& cfg,
    MpcWorkspace& ws
) noexcept {
    const std::size_t m = cfg.control_dim;
    [[maybe_unused]] const std::size_t N = cfg.horizon;

    const std::size_t n_ineq = 2U * m * N;

    if (n_ineq > kMpcMaxInequality) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{0U}; i < m * N; ++i) {
        const std::size_t control_idx = i % m;

        ws.A_ineq.at(i, i) = scalar{1.0f};
        ws.b_ineq.at(i) = cfg.u_max.at(control_idx);

        ws.A_ineq.at(m * N + i, i) = scalar{-1.0f};
        ws.b_ineq.at(m * N + i) = -cfg.u_min.at(control_idx);
    }

    return true;
}

// ═══════════════════════════════════════════
// QP SOLVER — Projected Gradient
// ═══════════════════════════════════════════

[[nodiscard]] inline bool mpc_qp_solve(
    const MpcWorkspace& ws,
    const std::size_t nu_total,
    const std::size_t n_ineq,
    Vector_Dynamic<kMpcMaxControl * kMpcMaxHorizon>& u_optimal
) noexcept {
    if (nu_total == 0U || nu_total > kMpcMaxControl * kMpcMaxHorizon) {
        APSA_UNLIKELY { return false; }
    }

    u_optimal = Vector_Dynamic<kMpcMaxControl * kMpcMaxHorizon>{nu_total};

    for (std::size_t iter{0U}; iter < kMpcMaxQpIterations; ++iter) {
        Vector_Dynamic<kMpcMaxControl * kMpcMaxHorizon> gradient{nu_total};

        for (std::size_t i{0U}; i < nu_total; ++i) {
            scalar sum{0.0f};

            for (std::size_t j{0U}; j < nu_total; ++j) {
                sum += ws.H.at(i, j) * u_optimal.at(j);
            }

            gradient.at(i) = sum + ws.f.at(i);
        }

        const scalar step_size = scalar{1.0f} /
            (ws.H.at(0, 0) + kMpcRegularization);

        Vector_Dynamic<kMpcMaxControl * kMpcMaxHorizon> u_new{nu_total};

        for (std::size_t i{0U}; i < nu_total; ++i) {
            u_new.at(i) = u_optimal.at(i) - step_size * gradient.at(i);
        }

        for (std::size_t c{0U}; c < n_ineq; ++c) {
            scalar constraint_value{0.0f};

            for (std::size_t j{0U}; j < nu_total; ++j) {
                constraint_value += ws.A_ineq.at(c, j) * u_new.at(j);
            }

            if (constraint_value > ws.b_ineq.at(c)) {
                const scalar excess = constraint_value - ws.b_ineq.at(c);

                scalar norm_row{0.0f};

                for (std::size_t j{0U}; j < nu_total; ++j) {
                    norm_row += ws.A_ineq.at(c, j) * ws.A_ineq.at(c, j);
                }

                if (norm_row > kMpcRegularization) {
                    const scalar correction = excess / norm_row;

                    for (std::size_t j{0U}; j < nu_total; ++j) {
                        u_new.at(j) -= correction * ws.A_ineq.at(c, j);
                    }
                }
            }
        }

        scalar diff{0.0f};

        for (std::size_t i{0U}; i < nu_total; ++i) {
            const scalar d = u_new.at(i) - u_optimal.at(i);
            diff += d * d;
        }

        u_optimal = u_new;

        if (std::sqrt(diff) < kMpcQpTolerance) {
            return true;
        }
    }

    return false;
}

// ═══════════════════════════════════════════
// MPC MAIN STEP
// ═══════════════════════════════════════════

[[nodiscard]] inline bool mpc_step(
    const MpcConfig& cfg,
    const Vector_Dynamic<kMpcMaxState>& x_current,
    MpcWorkspace& ws,
    Vector_Dynamic<kMpcMaxControl>& u_command
) noexcept {
    [[maybe_unused]] const std::size_t n = cfg.state_dim;
    const std::size_t m = cfg.control_dim;
    [[maybe_unused]] const std::size_t N = cfg.horizon;

    if (!mpc_build_prediction(cfg, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!mpc_build_cost(cfg, x_current, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!mpc_build_constraints(cfg, ws)) APSA_UNLIKELY {
        return false;
    }

    Vector_Dynamic<kMpcMaxControl * kMpcMaxHorizon> u_optimal{m * N};

    if (!mpc_qp_solve(ws, m * N, 2U * m * N, u_optimal)) APSA_UNLIKELY {
        return false;
    }

    u_command = Vector_Dynamic<kMpcMaxControl>{m};

    for (std::size_t i{0U}; i < m; ++i) {
        u_command.at(i) = u_optimal.at(i);
    }

    return true;
}

// ═══════════════════════════════════════════
// TUBE MPC CONFIG
// ═══════════════════════════════════════════

struct TubeMpcConfig {
    std::size_t state_dim{0U};
    std::size_t control_dim{0U};
    std::size_t horizon{0U};

    Matrix_Dynamic<kTubeMpcMaxState, kTubeMpcMaxState> Phi;
    Matrix_Dynamic<kTubeMpcMaxState, kTubeMpcMaxControl> Gamma;
    Matrix_Dynamic<kTubeMpcMaxState, kTubeMpcMaxState> Q;
    Matrix_Dynamic<kTubeMpcMaxControl, kTubeMpcMaxControl> R;
    Matrix_Dynamic<kTubeMpcMaxState, kTubeMpcMaxState> P_terminal;
    Vector_Dynamic<kTubeMpcMaxControl> u_min;
    Vector_Dynamic<kTubeMpcMaxControl> u_max;
    Vector_Dynamic<kTubeMpcMaxState> w_max;
    Matrix_Dynamic<kTubeMpcMaxControl, kTubeMpcMaxState> K_tube;
    
    explicit TubeMpcConfig(std::size_t n = kTubeMpcMaxState,
                           std::size_t m = kTubeMpcMaxControl) noexcept
        : Phi(n, n)
        , Gamma(n, m)
        , Q(n, n)
        , R(m, m)
        , P_terminal(n, n)
        , u_min(m)
        , u_max(m)
        , w_max(n)
        , K_tube(m, n)
    {}
};

// ═══════════════════════════════════════════
// TUBE MPC WORKSPACE
// ═══════════════════════════════════════════

struct TubeMpcWorkspace {
    MpcWorkspace nominal_ws;
    Vector_Dynamic<kTubeMpcMaxControl> u_min_tight;
    Vector_Dynamic<kTubeMpcMaxControl> u_max_tight;
    Vector_Dynamic<kTubeMpcMaxState> tube_radius;
    
    explicit TubeMpcWorkspace(
        std::size_t n = kTubeMpcMaxState,
        std::size_t m = kTubeMpcMaxControl,
        std::size_t N = kTubeMpcMaxHorizon
    ) noexcept
        : nominal_ws(n, m, N)
        , u_min_tight(m)
        , u_max_tight(m)
        , tube_radius(n)
    {}
};

// ═══════════════════════════════════════════
// COMPUTE MINIMAL RPI
// ═══════════════════════════════════════════

[[nodiscard]] inline bool tube_mpc_compute_mrpi(
    const TubeMpcConfig& cfg,
    TubeMpcWorkspace& ws
) noexcept {
    const std::size_t n = cfg.state_dim;

    if (n == 0U || n > kTubeMpcMaxState) APSA_UNLIKELY {
        return false;
    }

    const auto GK = cfg.Gamma * cfg.K_tube;
    [[maybe_unused]] const auto Phi_cl = cfg.Phi + GK;

    const scalar contraction = kTubeMpcAlpha;

    if (contraction >= scalar{1.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar inv_factor = scalar{1.0f} / (scalar{1.0f} - contraction);

    for (std::size_t i{0U}; i < n; ++i) {
        ws.tube_radius.at(i) = cfg.w_max.at(i) * inv_factor;
    }

    return true;
}

// ═══════════════════════════════════════════
// TIGHTEN INPUT CONSTRAINTS
// ═══════════════════════════════════════════

[[nodiscard]] inline bool tube_mpc_tighten_constraints(
    const TubeMpcConfig& cfg,
    TubeMpcWorkspace& ws
) noexcept {
    const std::size_t m = cfg.control_dim;
    const std::size_t n = cfg.state_dim;

    if (m == 0U || m > kTubeMpcMaxControl) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{0U}; i < m; ++i) {
        scalar tightening{0.0f};

        for (std::size_t j{0U}; j < n; ++j) {
            tightening += std::abs(cfg.K_tube.at(i, j)) * ws.tube_radius.at(j);
        }

        ws.u_min_tight.at(i) = cfg.u_min.at(i) + tightening;
        ws.u_max_tight.at(i) = cfg.u_max.at(i) - tightening;

        if (ws.u_min_tight.at(i) > ws.u_max_tight.at(i)) APSA_UNLIKELY {
            return false;
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// ROBUST MPC MAIN
// ═══════════════════════════════════════════

[[nodiscard]] inline bool tube_mpc_step(
    const TubeMpcConfig& cfg,
    const Vector_Dynamic<kTubeMpcMaxState>& x_current,
    TubeMpcWorkspace& ws,
    Vector_Dynamic<kTubeMpcMaxControl>& u_command
) noexcept {
    const std::size_t n = cfg.state_dim;
    const std::size_t m = cfg.control_dim;
    [[maybe_unused]] const std::size_t N = cfg.horizon;

    if (!tube_mpc_compute_mrpi(cfg, ws)) APSA_UNLIKELY {
        return false;
    }

    if (!tube_mpc_tighten_constraints(cfg, ws)) APSA_UNLIKELY {
        return false;
    }

    MpcConfig nominal_cfg{};

    nominal_cfg.state_dim = cfg.state_dim;
    nominal_cfg.control_dim = cfg.control_dim;
    nominal_cfg.horizon = cfg.horizon;
    nominal_cfg.Phi = cfg.Phi;
    nominal_cfg.Gamma = cfg.Gamma;
    nominal_cfg.Q = cfg.Q;
    nominal_cfg.R = cfg.R;
    nominal_cfg.P_terminal = cfg.P_terminal;
    nominal_cfg.u_min = ws.u_min_tight;
    nominal_cfg.u_max = ws.u_max_tight;

    Vector_Dynamic<kTubeMpcMaxControl> u_nominal{m};

    if (!mpc_step(nominal_cfg, x_current, ws.nominal_ws, u_nominal)) {
        APSA_UNLIKELY { return false; }
    }

    Vector_Dynamic<kTubeMpcMaxControl> u_feedback{m};

    for (std::size_t i{0U}; i < m; ++i) {
        scalar fb{0.0f};

        for (std::size_t j{0U}; j < n; ++j) {
            fb += cfg.K_tube.at(i, j) * x_current.at(j);
        }

        u_feedback.at(i) = fb;
    }

    u_command = u_nominal + u_feedback;

    return true;
}