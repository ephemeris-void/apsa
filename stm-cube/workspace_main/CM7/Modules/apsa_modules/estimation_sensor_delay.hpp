#pragma once

#include "apsa_estimation_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <cstddef>

// ═══════════════════════════════════════════
// APSA ESTIMATION — SENSOR DELAY COMPENSATION
// Out-of-sequence measurement handling
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// SENSOR DELAY CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kDelayMaxState{21};
constexpr std::size_t kDelayMaxMeasurement{9};
constexpr std::size_t kDelayMaxSteps{50};
constexpr scalar kDelaySearchTolerance{1e-6};

// ═══════════════════════════════════════════
// SENSOR DELAY CONFIG
// ═══════════════════════════════════════════

struct SensorDelayConfig {
    Time known_delay{0.0f};
    Time sample_period{0.0f};
    std::size_t max_delay_steps{10};
};

// ═══════════════════════════════════════════
// SENSOR DELAY STATE
// ═══════════════════════════════════════════

struct SensorDelayState {
    std::size_t delay_steps{0U};
    scalar effective_delay{0.0f};
    bool delay_estimated{false};
    bool initialized{false};
};

// ═══════════════════════════════════════════
// SENSOR DELAY WORKSPACE
// ═══════════════════════════════════════════

struct SensorDelayWorkspace {
    Matrix_Dynamic<kDelayMaxState, kDelayMaxState> Phi_power;
    Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxState> H_delayed;
    Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxMeasurement> R_eff;
    
    explicit SensorDelayWorkspace(
        std::size_t n = kDelayMaxState,
        std::size_t m = kDelayMaxMeasurement
    ) noexcept
        : Phi_power(n, n)
        , H_delayed(m, n)
        , R_eff(m, m)
    {}
};

// ═══════════════════════════════════════════
// COMPUTE DELAY STEPS — N = τ_d / T_s
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool sensor_delay_steps(
    const SensorDelayConfig& cfg,
    SensorDelayState& state
) noexcept {
    if (cfg.sample_period.get() <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (cfg.known_delay.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar ratio = cfg.known_delay.get() / cfg.sample_period.get();

    state.delay_steps = static_cast<std::size_t>(std::round(ratio));

    if (state.delay_steps > kDelayMaxSteps) APSA_UNLIKELY {
        return false;
    }

    state.effective_delay =
        static_cast<scalar>(state.delay_steps) * cfg.sample_period.get();

    state.delay_estimated = true;
    state.initialized = true;

    return true;
}

// ═══════════════════════════════════════════
// COMPUTE Phi^N — State Transition Power
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool sensor_delay_phi_power(
    const Matrix_Dynamic<kDelayMaxState, kDelayMaxState>& Phi,
    const std::size_t power,
    SensorDelayWorkspace& ws
) noexcept {
    if (power > kDelayMaxSteps) APSA_UNLIKELY {
        return false;
    }

    const std::size_t n = Phi.rows();

    ws.Phi_power = Matrix_Dynamic<kDelayMaxState, kDelayMaxState>{n, n};

    for (std::size_t i{0U}; i < n; ++i) {
        ws.Phi_power.at(i, i) = scalar{1.0f};
    }

    for (std::size_t k{0U}; k < power; ++k) {
        ws.Phi_power = ws.Phi_power * Phi;
    }

    return true;
}

// ═══════════════════════════════════════════
// COMPUTE DELAYED H — H_delayed = H · Φ^N
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool sensor_delay_H_delayed(
    const Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxState>& H,
    const SensorDelayWorkspace& ws,
    Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxState>& H_del
) noexcept {
    H_del = H * ws.Phi_power;

    return true;
}

// ═══════════════════════════════════════════
// COMPUTE EFFECTIVE R
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool sensor_delay_R_effective(
    const Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxMeasurement>& R,
    const Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxState>& H_del,
    const Matrix_Dynamic<kDelayMaxState, kDelayMaxState>& P_pred,
    Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxMeasurement>& R_eff
) noexcept {
    const auto H_del_T = transpose(H_del);
    const auto HPH_T = H_del * P_pred * H_del_T;

    R_eff = R + HPH_T;
    R_eff = (R_eff + transpose(R_eff)) * scalar{0.5f};

    if (!is_positive_definite(R_eff)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// STATE PREDICTION TO DELAYED TIME
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool sensor_delay_state_prediction(
    const Vector_Dynamic<kDelayMaxState>& state_current,
    const SensorDelayWorkspace& ws,
    Vector_Dynamic<kDelayMaxState>& state_delayed
) noexcept {
    state_delayed = ws.Phi_power * state_current;

    return true;
}

// ═══════════════════════════════════════════
// FULL DELAY COMPENSATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool sensor_delay_compensate(
    const SensorDelayConfig& cfg,
    const Matrix_Dynamic<kDelayMaxState, kDelayMaxState>& Phi,
    const Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxState>& H,
    const Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxMeasurement>& R,
    const Matrix_Dynamic<kDelayMaxState, kDelayMaxState>& P_pred,
    const Vector_Dynamic<kDelayMaxState>& state_current,
    SensorDelayState& delay_state,
    SensorDelayWorkspace& ws,
    Vector_Dynamic<kDelayMaxState>& state_delayed,
    Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxState>& H_delayed_out,
    Matrix_Dynamic<kDelayMaxMeasurement, kDelayMaxMeasurement>& R_eff_out
) noexcept {
    if (!sensor_delay_steps(cfg, delay_state)) APSA_UNLIKELY {
        return false;
    }

    if (!sensor_delay_phi_power(Phi, delay_state.delay_steps, ws)) {
        APSA_UNLIKELY { return false; }
    }

    if (!sensor_delay_H_delayed(H, ws, H_delayed_out)) {
        APSA_UNLIKELY { return false; }
    }

    if (!sensor_delay_R_effective(R, H_delayed_out, P_pred, R_eff_out)) {
        APSA_UNLIKELY { return false; }
    }

    if (!sensor_delay_state_prediction(state_current, ws, state_delayed)) {
        APSA_UNLIKELY { return false; }
    }

    return true;
}