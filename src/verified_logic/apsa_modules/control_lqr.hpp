#pragma once

#include "apsa_control_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA CONTROL — LQR
// Linear Quadratic Regulator
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// LQR DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kLqrMaxState{21};
constexpr std::size_t kLqrMaxControl{6};

// ═══════════════════════════════════════════
// LQR CONVERGENCE CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kLqrMaxIterations{100};
constexpr scalar kLqrConvergenceTolerance{1e-10};

// ═══════════════════════════════════════════
// THRESHOLD CONSTANTS
// ═══════════════════════════════════════════


// ═══════════════════════════════════════════
// LQR RESULT
// ═══════════════════════════════════════════

struct LqrResult {
    Matrix_Dynamic<kLqrMaxState, kLqrMaxState> P;
    Matrix_Dynamic<kLqrMaxControl, kLqrMaxState> K;
    scalar cost{0.0f};
    bool converged{false};
    
    explicit LqrResult(
        std::size_t n = kLqrMaxState,
        std::size_t m = kLqrMaxControl
    ) noexcept
        : P(n, n)
        , K(m, n)
    {}
};

// ═══════════════════════════════════════════
// BRYSON'S RULE — Q and R
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bryson_rule_QR(
    const Vector_Dynamic<kLqrMaxState>& x_max,
    const Vector_Dynamic<kLqrMaxControl>& u_max,
    Matrix_Dynamic<kLqrMaxState, kLqrMaxState>& Q,
    Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl>& R
) noexcept {
    const std::size_t n = x_max.size();
    const std::size_t m = u_max.size();

    if (n == 0U || n > kLqrMaxState) APSA_UNLIKELY {
        return false;
    }

    if (m == 0U || m > kLqrMaxControl) APSA_UNLIKELY {
        return false;
    }

    Q = Matrix_Dynamic<kLqrMaxState, kLqrMaxState>{n, n};
    R = Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl>{m, m};

    for (std::size_t i{0U}; i < n; ++i) {
        if (std::abs(x_max.at(i)) < kLqrConvergenceTolerance) {
            APSA_UNLIKELY { return false; }
        }

        Q.at(i, i) = scalar{1.0f} / (x_max.at(i) * x_max.at(i));
    }

    for (std::size_t i{0U}; i < m; ++i) {
        if (std::abs(u_max.at(i)) < kLqrConvergenceTolerance) {
            APSA_UNLIKELY { return false; }
        }

        R.at(i, i) = scalar{1.0f} / (u_max.at(i) * u_max.at(i));
    }

    return true;
}

// ═══════════════════════════════════════════
// CONTINUOUS ALGEBRAIC RICCATI — Kleinman
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool lqr_solve_care(
    const Matrix_Dynamic<kLqrMaxState, kLqrMaxState>& A,
    const Matrix_Dynamic<kLqrMaxState, kLqrMaxControl>& B,
    const Matrix_Dynamic<kLqrMaxState, kLqrMaxState>& Q,
    const Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl>& R,
    LqrResult& result
) noexcept {
    const std::size_t n = A.rows();
    const std::size_t m = B.cols();

    if (n == 0U || n > kLqrMaxState) APSA_UNLIKELY {
        return false;
    }

    if (m == 0U || m > kLqrMaxControl) APSA_UNLIKELY {
        return false;
    }

    if (A.rows() != n || A.cols() != n) APSA_UNLIKELY {
        return false;
    }

    if (B.rows() != n || B.cols() != m) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(Q)) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(R)) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl> R_chol{m, m};

    if (!cholesky(R, R_chol)) APSA_UNLIKELY {
        return false;
    }

    result.P = Q;

    for (std::size_t iter{0U}; iter < kLqrMaxIterations; ++iter) {
        const auto BT_P = transpose(B) * result.P;

        Matrix_Dynamic<kLqrMaxControl, kLqrMaxState> K_new{m, n};

        for (std::size_t col{0U}; col < n; ++col) {
            Vector_Dynamic<kLqrMaxControl> b_col{m};

            for (std::size_t i{0U}; i < m; ++i) {
                b_col.at(i) = BT_P.at(i, col);
            }

            Vector_Dynamic<kLqrMaxControl> y_col{m};

            if (!forward_substitute(b_col, R_chol, y_col)) APSA_UNLIKELY {
                return false;
            }

            const auto R_chol_T = transpose(R_chol);

            Vector_Dynamic<kLqrMaxControl> x_col{m};

            if (!backward_substitute(y_col, R_chol_T, x_col)) APSA_UNLIKELY {
                return false;
            }

            for (std::size_t i{0U}; i < m; ++i) {
                K_new.at(i, col) = x_col.at(i);
            }
        }

        const auto BK = B * K_new;
        const auto A_cl = A - BK;
        const auto A_cl_T = transpose(A_cl);
        const auto KRK = transpose(K_new) * R * K_new;
        const auto rhs = Q + KRK;

        // Iterative Lyapunov: A_cl^T P + P A_cl = -rhs
        Matrix_Dynamic<kLqrMaxState, kLqrMaxState> P_new = result.P;
        
        // Compute step size from ||A_cl||_inf
        scalar norm_A{0.0f};
        for (std::size_t i{0U}; i < n; ++i) {
            scalar row_sum{0.0f};
            for (std::size_t j{0U}; j < n; ++j) {
                row_sum += std::abs(A_cl.at(i, j));
            }
            if (row_sum > norm_A) norm_A = row_sum;
        }
        
        if (norm_A < kLqrConvergenceTolerance) {
            // A_cl ≈ 0 → P = rhs / 0 → use direct
            P_new = rhs;
        } else {
            const scalar alpha = scalar{1.0f} / (scalar{2.0f} * norm_A);
            
            for (std::size_t inner{0U}; inner < kLqrMaxIterations * 10U; ++inner) {
                // Residual = A_cl^T P + P A_cl + rhs
                Matrix_Dynamic<kLqrMaxState, kLqrMaxState> resid{n, n};
                for (std::size_t i{0U}; i < n; ++i) {
                    for (std::size_t j{0U}; j < n; ++j) {
                        scalar sum{0.0f};
                        for (std::size_t k{0U}; k < n; ++k) {
                            sum += A_cl_T.at(i, k) * P_new.at(k, j);
                            sum += P_new.at(i, k) * A_cl.at(k, j);
                        }
                        resid.at(i, j) = sum + rhs.at(i, j);
                    }
                }
                
                scalar resid_norm{0.0f};
                for (std::size_t i{0U}; i < n; ++i) {
                    for (std::size_t j{0U}; j < n; ++j) {
                        resid_norm += resid.at(i, j) * resid.at(i, j);
                    }
                }
                resid_norm = std::sqrt(resid_norm);
                
                if (resid_norm < kLqrConvergenceTolerance) {
                    break;
                }
                
                // P += alpha * residual
                for (std::size_t i{0U}; i < n; ++i) {
                    for (std::size_t j{0U}; j < n; ++j) {
                        P_new.at(i, j) += alpha * resid.at(i, j);
                    }
                }
            }
        }
        
        // Symmetrize
        P_new = (P_new + transpose(P_new)) * scalar{0.5f};

        scalar total_diff{0.0f};

        for (std::size_t i{0U}; i < n; ++i) {
            for (std::size_t j{0U}; j < n; ++j) {
                const scalar d = P_new.at(i, j) - result.P.at(i, j);
                total_diff += d * d;
            }
        }

        result.P = P_new;
        result.K = K_new;

        if (std::sqrt(total_diff) < kLqrConvergenceTolerance) {
            result.converged = true;
            break;
        }
    }

    if (!result.converged) APSA_UNLIKELY {
        return false;
    }

    scalar cost{0.0f};

    for (std::size_t i{0U}; i < n; ++i) {
        cost += result.P.at(i, i);
    }

    result.cost = cost;

    return true;
}

// ═══════════════════════════════════════════
// DISCRETE ALGEBRAIC RICCATI EQUATION
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool lqr_solve_dare(
    const Matrix_Dynamic<kLqrMaxState, kLqrMaxState>& A,
    const Matrix_Dynamic<kLqrMaxState, kLqrMaxControl>& B,
    const Matrix_Dynamic<kLqrMaxState, kLqrMaxState>& Q,
    const Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl>& R,
    LqrResult& result
) noexcept {
    const std::size_t n = A.rows();
    const std::size_t m = B.cols();

    if (n == 0U || n > kLqrMaxState) APSA_UNLIKELY {
        return false;
    }

    if (m == 0U || m > kLqrMaxControl) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(Q)) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(R)) APSA_UNLIKELY {
        return false;
    }

    result.P = Q;

    for (std::size_t iter{0U}; iter < kLqrMaxIterations; ++iter) {
        const auto BT = transpose(B);
        const auto BT_P = BT * result.P;
        const auto BT_P_B = BT_P * B;
        const auto R_plus_BTPB = R + BT_P_B;

        Matrix_Dynamic<kLqrMaxControl, kLqrMaxControl> L{m, m};

        if (!cholesky(R_plus_BTPB, L)) APSA_UNLIKELY {
            return false;
        }

        const auto BT_P_A = BT_P * A;

        Matrix_Dynamic<kLqrMaxControl, kLqrMaxState> K_new{m, n};

        for (std::size_t col{0U}; col < n; ++col) {
            Vector_Dynamic<kLqrMaxControl> b_col{m};

            for (std::size_t i{0U}; i < m; ++i) {
                b_col.at(i) = BT_P_A.at(i, col);
            }

            Vector_Dynamic<kLqrMaxControl> y_col{m};

            if (!forward_substitute(b_col, L, y_col)) APSA_UNLIKELY {
                return false;
            }

            const auto L_T = transpose(L);

            Vector_Dynamic<kLqrMaxControl> x_col{m};

            if (!backward_substitute(y_col, L_T, x_col)) APSA_UNLIKELY {
                return false;
            }

            for (std::size_t i{0U}; i < m; ++i) {
                K_new.at(i, col) = x_col.at(i);
            }
        }

        const auto AT_P_A = transpose(A) * result.P * A;
        const auto K_RK = transpose(K_new) * R_plus_BTPB * K_new;
        const auto P_new = AT_P_A - K_RK + Q;

        scalar diff{0.0f};

        for (std::size_t i{0U}; i < n; ++i) {
            for (std::size_t j{0U}; j < n; ++j) {
                const scalar d = P_new.at(i, j) - result.P.at(i, j);
                diff += d * d;
            }
        }

        result.P = P_new;
        result.K = K_new;

        if (std::sqrt(diff) < kLqrConvergenceTolerance) {
            result.converged = true;
            break;
        }
    }

    if (!result.converged) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// LQR CONTROL LAW
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool lqr_control(
    const Matrix_Dynamic<kLqrMaxControl, kLqrMaxState>& K,
    const Vector_Dynamic<kLqrMaxState>& x,
    Vector_Dynamic<kLqrMaxControl>& u
) noexcept {
    const std::size_t m = K.rows();
    const std::size_t n = K.cols();

    if (n == 0U || m == 0U) APSA_UNLIKELY {
        return false;
    }

    u = Vector_Dynamic<kLqrMaxControl>{m};

    for (std::size_t i{0U}; i < m; ++i) {
        scalar sum{0.0f};

        for (std::size_t j{0U}; j < n; ++j) {
            sum += K.at(i, j) * x.at(j);
        }

        u.at(i) = -sum;
    }

    return true;
}