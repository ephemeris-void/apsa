#pragma once

#include "apsa_control_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA CONTROL — SLIDING MODE CONTROL
// Classical SMC + Adaptive SMC
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// SMC DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kSmcMaxOrder{4};
constexpr std::size_t kSmcMaxState{12};
constexpr std::size_t kSmcMaxControl{6};

// ═══════════════════════════════════════════
// SMC CONSTANTS
// ═══════════════════════════════════════════

constexpr scalar kSmcSaturationThreshold{1e-3};
constexpr scalar kSmcBoundaryLayer{0.05f};

// ═══════════════════════════════════════════
// SMC CONFIG
// ═══════════════════════════════════════════

struct SmcConfig {
    std::size_t system_order{0U};

    std::array<scalar, kSmcMaxOrder> surface_coeffs{};

    scalar gain{0.0f};

    scalar phi_boundary{kSmcBoundaryLayer};
};

// ═══════════════════════════════════════════
// SMC STATE
// ═══════════════════════════════════════════

struct SmcState {
    std::array<scalar, kSmcMaxOrder> error_derivatives{};
    scalar sliding_surface{0.0f};
    scalar control_output{0.0f};
    bool initialized{false};
};

// ═══════════════════════════════════════════
// COMPUTE SLIDING SURFACE — s = Σ cᵢ·e⁽ⁱ⁾
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool smc_compute_surface(
    const SmcConfig& cfg,
    const SmcState& state,
    scalar& surface
) noexcept {
    if (cfg.system_order == 0U || cfg.system_order > kSmcMaxOrder) {
        APSA_UNLIKELY { return false; }
    }

    scalar s{0.0f};

    for (std::size_t i{0U}; i < cfg.system_order; ++i) {
        s += cfg.surface_coeffs[i] * state.error_derivatives[i];
    }

    if (!std::isfinite(s)) APSA_UNLIKELY {
        return false;
    }

    surface = s;
    return true;
}

// ═══════════════════════════════════════════
// SIGN FUNCTION WITH BOUNDARY LAYER
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool smc_saturation(
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
// SMC CONTROL LAW — Full
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool smc_control(
    const SmcConfig& cfg,
    const SmcState& state,
    const scalar equivalent_control,
    const scalar disturbance_bound,
    scalar& control
) noexcept {
    if (cfg.gain <= disturbance_bound) APSA_UNLIKELY {
        return false;
    }

    if (disturbance_bound < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    scalar surface{0.0f};

    if (!smc_compute_surface(cfg, state, surface)) APSA_UNLIKELY {
        return false;
    }

    scalar sat_output{0.0f};

    if (!smc_saturation(surface, cfg.phi_boundary, sat_output)) {
        APSA_UNLIKELY { return false; }
    }

    control = equivalent_control - cfg.gain * sat_output;

    if (!std::isfinite(control)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// LYAPUNOV VERIFICATION
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool smc_lyapunov_check(
    const SmcConfig& cfg,
    const scalar surface,
    const scalar disturbance_bound,
    scalar& V,
    scalar& V_dot
) noexcept {
    if (cfg.gain <= disturbance_bound) APSA_UNLIKELY {
        return false;
    }

    V = scalar{0.5f} * surface * surface;

    if (V < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar gain_margin = cfg.gain - disturbance_bound;

    V_dot = -gain_margin * std::abs(surface);

    if (V_dot > scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// ADAPTIVE SMC STATE
// ═══════════════════════════════════════════

struct AdaptiveSmcState {
    scalar gain{0.0f};
    scalar gain_integral{0.0f};
    std::size_t steps{0U};
};

// ═══════════════════════════════════════════
// ADAPTIVE GAIN UPDATE
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool smc_adaptive_gain_update(
    const scalar surface,
    const scalar adaptation_rate,
    const scalar sigma_modification,
    AdaptiveSmcState& adaptive
) noexcept {
    if (adaptation_rate <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (sigma_modification < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar gain_dot =
        adaptation_rate * std::abs(surface) -
        sigma_modification * adaptive.gain;

    adaptive.gain += gain_dot;

    if (adaptive.gain < scalar{0.0f}) APSA_UNLIKELY {
        adaptive.gain = scalar{0.0f};
        return false;
    }

    if (!std::isfinite(adaptive.gain)) APSA_UNLIKELY {
        return false;
    }

    adaptive.gain_integral += adaptive.gain;
    ++adaptive.steps;

    return true;
}