#pragma once

#include "apsa_control_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

// ═══════════════════════════════════════════
// APSA CONTROL — EXTENDED SLIDING MODE
// Super-Twisting + Finite-Time + Fixed-Time
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// SUPER-TWISTING DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kSuperTwistMaxState{4};
constexpr std::size_t kSuperTwistMaxControl{6};

// ═══════════════════════════════════════════
// SUPER-TWISTING CONSTANTS
// ═══════════════════════════════════════════

constexpr scalar kSuperTwistTolerance{1e-10};
constexpr scalar kSuperTwistBoundary{1e-4};

// ═══════════════════════════════════════════
// FINITE-TIME CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kFiniteTimeMaxState{4};
constexpr std::size_t kFiniteTimeMaxControl{6};
constexpr scalar kFiniteTimeEpsilon{1e-12};

// ═══════════════════════════════════════════
// FIXED-TIME CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kFixedTimeMaxState{4};
constexpr std::size_t kFixedTimeMaxControl{6};
constexpr scalar kFixedTimeEpsilon{1e-12};

// ═══════════════════════════════════════════
// SUPER-TWISTING CONFIG
// ═══════════════════════════════════════════

struct SuperTwistConfig {
    scalar k1{0.0f};
    scalar k2{0.0f};
    scalar disturbance_lipschitz{0.0f};
    scalar boundary_layer{kSuperTwistBoundary};
};

// ═══════════════════════════════════════════
// SUPER-TWISTING STATE
// ═══════════════════════════════════════════

struct SuperTwistState {
    scalar sliding_surface{0.0f};
    scalar integral_term{0.0f};
    scalar control_output{0.0f};
    bool initialized{false};
};

// ═══════════════════════════════════════════
// VALIDATE GAIN CONDITIONS
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool super_twist_validate_gains(
    const SuperTwistConfig& cfg
) noexcept {
    if (cfg.disturbance_lipschitz < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.k2 <= cfg.disturbance_lipschitz) APSA_UNLIKELY {
        return false;
    }

    const scalar L = cfg.disturbance_lipschitz;
    const scalar k2 = cfg.k2;

    const scalar term = (L * (k2 + L)) / (k2 - L);

    if (term < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar required_k1 = std::sqrt(term);

    if (cfg.k1 <= required_k1) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// SIGN FUNCTION — Boundary Layer
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool super_twist_sgn(
    const scalar value,
    const scalar boundary,
    scalar& output
) noexcept {
    if (boundary <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (value > boundary) {
        output = scalar{1.0f};
    } else if (value < -boundary) {
        output = scalar{-1.0f};
    } else {
        output = value / boundary;
    }

    if (!std::isfinite(output)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// ABSOLUTE VALUE POWER — |s|^(1/2)
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool super_twist_sqrt_abs(
    const scalar value,
    scalar& output
) noexcept {
    const scalar abs_val = std::abs(value);

    output = std::sqrt(abs_val);

    if (!std::isfinite(output)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// SUPER-TWISTING CONTROL UPDATE
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool super_twist_update(
    const SuperTwistConfig& cfg,
    const scalar sliding_surface,
    const Time dt,
    SuperTwistState& state
) noexcept {
    if (!super_twist_validate_gains(cfg)) APSA_UNLIKELY {
        return false;
    }

    if (dt.get() <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    state.sliding_surface = sliding_surface;

    scalar sqrt_abs_s{0.0f};

    if (!super_twist_sqrt_abs(sliding_surface, sqrt_abs_s)) {
        APSA_UNLIKELY { return false; }
    }

    scalar sgn_s{0.0f};

    if (!super_twist_sgn(sliding_surface, cfg.boundary_layer, sgn_s)) {
        APSA_UNLIKELY { return false; }
    }

    const scalar u1 = -cfg.k1 * sqrt_abs_s * sgn_s;

    const scalar u1_dot = -cfg.k2 * sgn_s;

    state.integral_term += u1_dot * dt.get();

    if (!std::isfinite(state.integral_term)) APSA_UNLIKELY {
        return false;
    }

    state.control_output = u1 + state.integral_term;

    if (!std::isfinite(state.control_output)) APSA_UNLIKELY {
        return false;
    }

    state.initialized = true;

    return true;
}

// ═══════════════════════════════════════════
// LYAPUNOV VERIFICATION — Super-Twisting
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool super_twist_lyapunov(
    const SuperTwistConfig& cfg,
    const scalar sliding_surface,
    scalar& V,
    scalar& V_dot
) noexcept {
    if (!super_twist_validate_gains(cfg)) APSA_UNLIKELY {
        return false;
    }

    V = scalar{0.5f} * sliding_surface * sliding_surface;

    if (V < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar eta = cfg.k1 - std::sqrt(cfg.k2);

    if (eta <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    V_dot = -eta * std::sqrt(V);

    if (V_dot > scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// RESET — Super-Twisting
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool super_twist_reset(
    SuperTwistState& state
) noexcept {
    state.sliding_surface = scalar{0.0f};
    state.integral_term = scalar{0.0f};
    state.control_output = scalar{0.0f};
    state.initialized = false;

    return true;
}

// ═══════════════════════════════════════════
// FINITE-TIME CONFIG
// ═══════════════════════════════════════════

struct FiniteTimeConfig {
    std::size_t state_dim{0U};

    scalar alpha{0.5f};
    scalar beta{1.5f};

    scalar k1{0.0f};
    scalar k2{0.0f};

    scalar c{0.0f};

    scalar error_bound{1e-6};
};

// ═══════════════════════════════════════════
// FINITE-TIME STATE
// ═══════════════════════════════════════════

struct FiniteTimeState {
    Vector_Dynamic<kFiniteTimeMaxState> error;
    Vector_Dynamic<kFiniteTimeMaxControl> control;
    scalar lyapunov_value{0.0f};
    scalar settling_time_bound{0.0f};
    bool converged{false};
    bool initialized{false};
    
    explicit FiniteTimeState(std::size_t n = kFiniteTimeMaxState) noexcept
        : error(n), control(n) {}
};

// ═══════════════════════════════════════════
// SIGNED POWER — ⌈e⌋^α = |e|^α · sgn(e)
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool finite_time_signed_power(
    const scalar error,
    const scalar power,
    scalar& output
) noexcept {
    if (power <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar abs_e = std::abs(error);
    const scalar pow_val = std::pow(abs_e, power);

    if (!std::isfinite(pow_val)) APSA_UNLIKELY {
        return false;
    }

    if (error > scalar{0.0f}) {
        output = pow_val;
    } else if (error < scalar{0.0f}) {
        output = -pow_val;
    } else {
        output = scalar{0.0f};
    }

    return true;
}

// ═══════════════════════════════════════════
// VALIDATE CONFIG — Finite-Time
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool finite_time_validate(
    const FiniteTimeConfig& cfg
) noexcept {
    if (cfg.state_dim == 0U || cfg.state_dim > kFiniteTimeMaxState) {
        APSA_UNLIKELY { return false; }
    }

    if (cfg.alpha <= scalar{0.0f} || cfg.alpha >= scalar{1.0f}) {
        APSA_UNLIKELY { return false; }
    }

    if (cfg.beta <= scalar{1.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.k1 <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.k2 < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.c <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// FINITE-TIME CONTROL LAW
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool finite_time_control(
    const FiniteTimeConfig& cfg,
    const Vector_Dynamic<kFiniteTimeMaxState>& error,
    FiniteTimeState& state
) noexcept {
    if (!finite_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    if (error.size() != cfg.state_dim) APSA_UNLIKELY {
        return false;
    }

    state.error = error;

    state.control = Vector_Dynamic<kFiniteTimeMaxControl>{cfg.state_dim};

    for (std::size_t i{0U}; i < cfg.state_dim; ++i) {
        scalar pow_alpha{0.0f};
        scalar pow_beta{0.0f};

        if (!finite_time_signed_power(error.at(i), cfg.alpha, pow_alpha)) {
            APSA_UNLIKELY { return false; }
        }

        if (!finite_time_signed_power(error.at(i), cfg.beta, pow_beta)) {
            APSA_UNLIKELY { return false; }
        }

        state.control.at(i) = -cfg.k1 * pow_alpha - cfg.k2 * pow_beta;

        if (!std::isfinite(state.control.at(i))) APSA_UNLIKELY {
            return false;
        }
    }

    state.initialized = true;
    return true;
}

// ═══════════════════════════════════════════
// LYAPUNOV VALUE — V = ½·‖e‖²
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool finite_time_lyapunov(
    const FiniteTimeConfig& cfg,
    const FiniteTimeState& state,
    scalar& V
) noexcept {
    if (!finite_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    scalar v{0.0f};

    for (std::size_t i{0U}; i < cfg.state_dim; ++i) {
        v += scalar{0.5f} * state.error.at(i) * state.error.at(i);
    }

    if (v < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    V = v;
    return true;
}

// ═══════════════════════════════════════════
// SETTLING TIME BOUND — T ≤ V₀^(1-α)/(c·(1-α))
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool finite_time_settling_bound(
    const FiniteTimeConfig& cfg,
    const scalar V0,
    scalar& T_settle
) noexcept {
    if (!finite_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    if (V0 < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar denom = cfg.c * (scalar{1.0f} - cfg.alpha);

    if (denom <= kFiniteTimeEpsilon) APSA_UNLIKELY {
        return false;
    }

    const scalar V_pow = std::pow(V0, scalar{1.0f} - cfg.alpha);

    T_settle = V_pow / denom;

    if (!std::isfinite(T_settle)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// CONVERGENCE CHECK
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool finite_time_convergence_check(
    const FiniteTimeConfig& cfg,
    const FiniteTimeState& state
) noexcept {
    if (!finite_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    scalar norm_error{0.0f};

    for (std::size_t i{0U}; i < cfg.state_dim; ++i) {
        norm_error += state.error.at(i) * state.error.at(i);
    }

    norm_error = std::sqrt(norm_error);

    return norm_error <= cfg.error_bound;
}

// ═══════════════════════════════════════════
// FIXED-TIME CONFIG
// ═══════════════════════════════════════════

struct FixedTimeConfig {
    std::size_t state_dim{0U};

    scalar alpha{0.5f};
    scalar beta{1.5f};

    scalar k1{0.0f};
    scalar k2{0.0f};

    scalar c1{0.0f};
    scalar c2{0.0f};
};

// ═══════════════════════════════════════════
// FIXED-TIME STATE
// ═══════════════════════════════════════════

struct FixedTimeState {
    Vector_Dynamic<kFixedTimeMaxState> error;
    Vector_Dynamic<kFixedTimeMaxControl> control;
    scalar lyapunov_value{0.0f};
    scalar max_settling_time{0.0f};
    bool initialized{false};
    
    explicit FixedTimeState(std::size_t n = kFixedTimeMaxState) noexcept
        : error(n), control(n) {}
};

// ═══════════════════════════════════════════
// VALIDATE CONFIG — Fixed-Time
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fixed_time_validate(
    const FixedTimeConfig& cfg
) noexcept {
    if (cfg.state_dim == 0U || cfg.state_dim > kFixedTimeMaxState) {
        APSA_UNLIKELY { return false; }
    }

    if (cfg.alpha <= scalar{0.0f} || cfg.alpha >= scalar{1.0f}) {
        APSA_UNLIKELY { return false; }
    }

    if (cfg.beta <= scalar{1.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.k1 <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.k2 <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.c1 <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.c2 <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// SIGNED POWER — Fixed-Time
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fixed_time_signed_power(
    const scalar error,
    const scalar power,
    scalar& output
) noexcept {
    if (power <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar abs_e = std::abs(error);
    const scalar pow_val = std::pow(abs_e, power);

    if (!std::isfinite(pow_val)) APSA_UNLIKELY {
        return false;
    }

    if (error > scalar{0.0f}) {
        output = pow_val;
    } else if (error < scalar{0.0f}) {
        output = -pow_val;
    } else {
        output = scalar{0.0f};
    }

    return true;
}

// ═══════════════════════════════════════════
// FIXED-TIME CONTROL LAW
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fixed_time_control(
    const FixedTimeConfig& cfg,
    const Vector_Dynamic<kFixedTimeMaxState>& error,
    FixedTimeState& state
) noexcept {
    if (!fixed_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    if (error.size() != cfg.state_dim) APSA_UNLIKELY {
        return false;
    }

    state.error = error;

    state.control = Vector_Dynamic<kFixedTimeMaxControl>{cfg.state_dim};

    for (std::size_t i{0U}; i < cfg.state_dim; ++i) {
        scalar pow_alpha{0.0f};
        scalar pow_beta{0.0f};

        if (!fixed_time_signed_power(error.at(i), cfg.alpha, pow_alpha)) {
            APSA_UNLIKELY { return false; }
        }

        if (!fixed_time_signed_power(error.at(i), cfg.beta, pow_beta)) {
            APSA_UNLIKELY { return false; }
        }

        state.control.at(i) = -cfg.k1 * pow_alpha - cfg.k2 * pow_beta;

        if (!std::isfinite(state.control.at(i))) APSA_UNLIKELY {
            return false;
        }
    }

    state.initialized = true;
    return true;
}

// ═══════════════════════════════════════════
// MAX SETTLING TIME — Independent of IC
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fixed_time_max_settling(
    const FixedTimeConfig& cfg,
    scalar& T_max
) noexcept {
    if (!fixed_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    const scalar term1 =
        scalar{1.0f} / (cfg.c1 * (scalar{1.0f} - cfg.alpha));

    const scalar term2 =
        scalar{1.0f} / (cfg.c2 * (cfg.beta - scalar{1.0f}));

    T_max = term1 + term2;

    if (!std::isfinite(T_max)) APSA_UNLIKELY {
        return false;
    }

    if (T_max <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// LYAPUNOV — V = ½·Σ eᵢ²
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fixed_time_lyapunov(
    const FixedTimeConfig& cfg,
    const FixedTimeState& state,
    scalar& V
) noexcept {
    if (!fixed_time_validate(cfg)) APSA_UNLIKELY {
        return false;
    }

    scalar v{0.0f};

    for (std::size_t i{0U}; i < cfg.state_dim; ++i) {
        v += scalar{0.5f} * state.error.at(i) * state.error.at(i);
    }

    if (v < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    V = v;
    return true;
}

// ═══════════════════════════════════════════
// RESET — Fixed-Time
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fixed_time_reset(
    FixedTimeState& state
) noexcept {
    state.error = Vector_Dynamic<kFixedTimeMaxState>{kFixedTimeMaxState};
    state.control = Vector_Dynamic<kFixedTimeMaxControl>{kFixedTimeMaxControl};
    state.lyapunov_value = scalar{0.0f};
    state.max_settling_time = scalar{0.0f};
    state.initialized = false;

    return true;
}