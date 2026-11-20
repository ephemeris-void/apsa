#pragma once

#include "apsa_control_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA CONTROL — CONTROL ALLOCATION
// Pseudo-inverse + Weighted + Null-space
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// CONTROL ALLOCATION CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kAllocMaxActuators{13};
constexpr std::size_t kAllocMaxAxes{6};
constexpr scalar kAllocMinDet{1e-12};
constexpr scalar kAllocMinWeight{1e-9};
constexpr scalar kAllocRegularization{1e-9};

// ═══════════════════════════════════════════
// ALLOCATION RESULT
// ═══════════════════════════════════════════

struct AllocationResult {
    scalar u[kAllocMaxActuators]{};
    std::size_t actuator_count{0U};
    scalar residual_norm{0.0f};
    bool saturated{false};
    bool valid{false};
};

// ═══════════════════════════════════════════
// B·u — FORWARD TORQUE MAP
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool allocation_forward(
    const scalar B[kAllocMaxAxes][kAllocMaxActuators],
    const scalar u[kAllocMaxActuators],
    const std::size_t axis_count,
    const std::size_t actuator_count,
    scalar tau_out[kAllocMaxAxes]
) noexcept {
    if (axis_count == 0U || axis_count > kAllocMaxAxes) {
        APSA_UNLIKELY { return false; }
    }

    if (actuator_count == 0U ||
        actuator_count > kAllocMaxActuators) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{0U}; i < axis_count; ++i) {
        scalar sum{0.0f};

        for (std::size_t j{0U}; j < actuator_count; ++j) {
            sum += B[i][j] * u[j];
        }

        tau_out[i] = sum;

        if (!std::isfinite(sum)) APSA_UNLIKELY {
            return false;
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// PSEUDO-INVERSE — Minimum Norm
// u* = Bᵀ·(B·Bᵀ)⁻¹·τ
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool allocation_pseudo_inverse(
    const scalar B[kAllocMaxAxes][kAllocMaxActuators],
    const scalar tau[kAllocMaxAxes],
    const std::size_t axis_count,
    const std::size_t actuator_count,
    AllocationResult& result
) noexcept {
    if (axis_count == 0U || axis_count > kAllocMaxAxes) {
        APSA_UNLIKELY { return false; }
    }

    if (actuator_count == 0U ||
        actuator_count > kAllocMaxActuators) APSA_UNLIKELY {
        return false;
    }

    scalar M[kAllocMaxAxes][kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        for (std::size_t j{0U}; j < axis_count; ++j) {
            scalar sum{0.0f};

            for (std::size_t k{0U}; k < actuator_count; ++k) {
                sum += B[i][k] * B[j][k];
            }

            M[i][j] = sum;
        }
    }

    scalar L[kAllocMaxAxes][kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        for (std::size_t j{0U}; j <= i; ++j) {
            scalar sum{0.0f};

            for (std::size_t k{0U}; k < j; ++k) {
                sum += L[i][k] * L[j][k];
            }

            if (i == j) {
                const scalar diag = M[i][i] - sum;

                if (diag <= kAllocMinDet) APSA_UNLIKELY {
                    return false;
                }

                L[i][j] = std::sqrt(diag);
            } else {
                if (L[j][j] < kAllocMinDet) APSA_UNLIKELY {
                    return false;
                }

                L[i][j] = (M[i][j] - sum) / L[j][j];
            }
        }
    }

    scalar y[kAllocMaxAxes]{};
    scalar z[kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        scalar sum{0.0f};

        for (std::size_t k{0U}; k < i; ++k) {
            sum += L[i][k] * z[k];
        }

        if (L[i][i] < kAllocMinDet) APSA_UNLIKELY {
            return false;
        }

        z[i] = (tau[i] - sum) / L[i][i];
    }

    for (std::size_t ii{axis_count}; ii-- > 0U;) {
        scalar sum{0.0f};

        for (std::size_t k{ii + 1U}; k < axis_count; ++k) {
            sum += L[k][ii] * y[k];
        }

        if (L[ii][ii] < kAllocMinDet) APSA_UNLIKELY {
            return false;
        }

        y[ii] = (z[ii] - sum) / L[ii][ii];
    }

    for (std::size_t j{0U}; j < actuator_count; ++j) {
        scalar sum{0.0f};

        for (std::size_t i{0U}; i < axis_count; ++i) {
            sum += B[i][j] * y[i];
        }

        result.u[j] = sum;

        if (!std::isfinite(sum)) APSA_UNLIKELY {
            return false;
        }
    }

    result.actuator_count = actuator_count;
    result.valid = true;

    return true;
}

// ═══════════════════════════════════════════
// WEIGHTED PSEUDO-INVERSE
// u* = W⁻¹Bᵀ(BW⁻¹Bᵀ)⁻¹·τ
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool allocation_weighted_pseudo_inverse(
    const scalar B[kAllocMaxAxes][kAllocMaxActuators],
    const scalar W_inv[kAllocMaxActuators],
    const scalar tau[kAllocMaxAxes],
    const std::size_t axis_count,
    const std::size_t actuator_count,
    AllocationResult& result
) noexcept {
    if (axis_count == 0U || axis_count > kAllocMaxAxes) {
        APSA_UNLIKELY { return false; }
    }

    if (actuator_count == 0U ||
        actuator_count > kAllocMaxActuators) APSA_UNLIKELY {
        return false;
    }

    scalar M[kAllocMaxAxes][kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        for (std::size_t j{0U}; j < axis_count; ++j) {
            scalar sum{0.0f};

            for (std::size_t k{0U}; k < actuator_count; ++k) {
                sum += B[i][k] * W_inv[k] * B[j][k];
            }

            M[i][j] = sum;
        }
    }

    scalar L[kAllocMaxAxes][kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        for (std::size_t j{0U}; j <= i; ++j) {
            scalar sum{0.0f};

            for (std::size_t k{0U}; k < j; ++k) {
                sum += L[i][k] * L[j][k];
            }

            if (i == j) {
                const scalar diag = M[i][i] - sum;

                if (diag <= kAllocMinDet) APSA_UNLIKELY {
                    return false;
                }

                L[i][j] = std::sqrt(diag);
            } else {
                if (L[j][j] < kAllocMinDet) APSA_UNLIKELY {
                    return false;
                }

                L[i][j] = (M[i][j] - sum) / L[j][j];
            }
        }
    }

    scalar y[kAllocMaxAxes]{};
    scalar z[kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        scalar sum{0.0f};

        for (std::size_t k{0U}; k < i; ++k) {
            sum += L[i][k] * z[k];
        }

        if (L[i][i] < kAllocMinDet) APSA_UNLIKELY {
            return false;
        }

        z[i] = (tau[i] - sum) / L[i][i];
    }

    for (std::size_t ii{axis_count}; ii-- > 0U;) {
        scalar sum{0.0f};

        for (std::size_t k{ii + 1U}; k < axis_count; ++k) {
            sum += L[k][ii] * y[k];
        }

        if (L[ii][ii] < kAllocMinDet) APSA_UNLIKELY {
            return false;
        }

        y[ii] = (z[ii] - sum) / L[ii][ii];
    }

    for (std::size_t j{0U}; j < actuator_count; ++j) {
        scalar sum{0.0f};

        for (std::size_t i{0U}; i < axis_count; ++i) {
            sum += B[i][j] * y[i];
        }

        result.u[j] = W_inv[j] * sum;

        if (!std::isfinite(result.u[j])) APSA_UNLIKELY {
            return false;
        }
    }

    result.actuator_count = actuator_count;
    result.valid = true;

    return true;
}

// ═══════════════════════════════════════════
// NULL-SPACE PROJECTION — Secondary Objective
// u = u_PI + (I - B⁺B)·v
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool allocation_null_space(
    const scalar B[kAllocMaxAxes][kAllocMaxActuators],
    const scalar u_PI[kAllocMaxActuators],
    const scalar v[kAllocMaxActuators],
    const std::size_t axis_count,
    const std::size_t actuator_count,
    AllocationResult& result
) noexcept {
    if (axis_count == 0U || axis_count > kAllocMaxAxes) {
        APSA_UNLIKELY { return false; }
    }

    if (actuator_count == 0U ||
        actuator_count > kAllocMaxActuators) APSA_UNLIKELY {
        return false;
    }

    scalar M[kAllocMaxAxes][kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        for (std::size_t j{0U}; j < axis_count; ++j) {
            scalar sum{0.0f};

            for (std::size_t k{0U}; k < actuator_count; ++k) {
                sum += B[i][k] * B[j][k];
            }

            M[i][j] = sum;
        }
    }

    scalar L[kAllocMaxAxes][kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        for (std::size_t j{0U}; j <= i; ++j) {
            scalar sum{0.0f};

            for (std::size_t k{0U}; k < j; ++k) {
                sum += L[i][k] * L[j][k];
            }

            if (i == j) {
                const scalar diag = M[i][i] - sum;

                if (diag <= kAllocMinDet) APSA_UNLIKELY {
                    return false;
                }

                L[i][j] = std::sqrt(diag);
            } else {
                if (L[j][j] < kAllocMinDet) APSA_UNLIKELY {
                    return false;
                }

                L[i][j] = (M[i][j] - sum) / L[j][j];
            }
        }
    }

    scalar Bv[kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        scalar sum{0.0f};

        for (std::size_t j{0U}; j < actuator_count; ++j) {
            sum += B[i][j] * v[j];
        }

        Bv[i] = sum;
    }

    scalar y[kAllocMaxAxes]{};
    scalar z[kAllocMaxAxes]{};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        scalar sum{0.0f};

        for (std::size_t k{0U}; k < i; ++k) {
            sum += L[i][k] * z[k];
        }

        if (L[i][i] < kAllocMinDet) APSA_UNLIKELY {
            return false;
        }

        z[i] = (Bv[i] - sum) / L[i][i];
    }

    for (std::size_t ii{axis_count}; ii-- > 0U;) {
        scalar sum{0.0f};

        for (std::size_t k{ii + 1U}; k < axis_count; ++k) {
            sum += L[k][ii] * y[k];
        }

        if (L[ii][ii] < kAllocMinDet) APSA_UNLIKELY {
            return false;
        }

        y[ii] = (z[ii] - sum) / L[ii][ii];
    }

    scalar Bplus_v[kAllocMaxActuators]{};

    for (std::size_t j{0U}; j < actuator_count; ++j) {
        scalar sum{0.0f};

        for (std::size_t i{0U}; i < axis_count; ++i) {
            sum += B[i][j] * y[i];
        }

        Bplus_v[j] = sum;
    }

    for (std::size_t j{0U}; j < actuator_count; ++j) {
        result.u[j] = u_PI[j] + (v[j] - Bplus_v[j]);

        if (!std::isfinite(result.u[j])) APSA_UNLIKELY {
            return false;
        }
    }

    result.actuator_count = actuator_count;
    result.valid = true;

    return true;
}

// ═══════════════════════════════════════════
// RESIDUAL VERIFICATION
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool allocation_verify_residual(
    const scalar B[kAllocMaxAxes][kAllocMaxActuators],
    const scalar tau_commanded[kAllocMaxAxes],
    const AllocationResult& result,
    const std::size_t axis_count,
    scalar& residual_norm
) noexcept {
    if (!result.valid) APSA_UNLIKELY {
        return false;
    }

    scalar tau_actual[kAllocMaxAxes]{};

    if (!allocation_forward(B, result.u, axis_count,
                            result.actuator_count, tau_actual)) {
        APSA_UNLIKELY { return false; }
    }

    scalar sum_sq{0.0f};

    for (std::size_t i{0U}; i < axis_count; ++i) {
        const scalar diff = tau_actual[i] - tau_commanded[i];
        sum_sq += diff * diff;
    }

    residual_norm = std::sqrt(sum_sq);

    if (!std::isfinite(residual_norm)) APSA_UNLIKELY {
        return false;
    }

    return true;
}