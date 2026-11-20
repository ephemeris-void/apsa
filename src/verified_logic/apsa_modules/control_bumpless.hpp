#pragma once

#include "apsa_control_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>

// ═══════════════════════════════════════════
// APSA CONTROL — BUMPLESS TRANSFER
// Smooth controller switching
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// BUMPLESS TRANSFER CONSTANTS
// ═══════════════════════════════════════════

constexpr scalar kBumpMinFadeTime{1e-3};
constexpr scalar kBumpMaxFadeTime{10.0f};
constexpr scalar kBumpMinKp{1e-9};
constexpr scalar kBumpAlphaMin{0.0f};
constexpr scalar kBumpAlphaMax{1.0f};

// ═══════════════════════════════════════════
// BUMPLESS TRANSFER STATE
// ═══════════════════════════════════════════

struct BumplessTransferState {
    scalar u_old{0.0f};
    scalar u_new{0.0f};
    scalar u_blended{0.0f};
    scalar alpha{0.0f};
    Time time_since_switch{0.0f};
    Time fade_duration{0.0f};
    bool transfer_active{false};
    bool valid{false};
};

// ═══════════════════════════════════════════
// INTEGRATOR PRELOAD
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bumpless_integrator_preload(
    const scalar u_previous,
    const scalar kp_new,
    const scalar error_current,
    scalar& integrator_new
) noexcept {
    if (!std::isfinite(u_previous) ||
        !std::isfinite(kp_new) ||
        !std::isfinite(error_current)) APSA_UNLIKELY {
        return false;
    }

    if (std::abs(kp_new) < kBumpMinKp) APSA_UNLIKELY {
        return false;
    }

    integrator_new = u_previous - kp_new * error_current;

    if (!std::isfinite(integrator_new)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// ALPHA COMPUTATION
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bumpless_alpha(
    const Time time_since_switch,
    const Time fade_duration,
    scalar& alpha
) noexcept {
    if (fade_duration.get() < kBumpMinFadeTime) APSA_UNLIKELY {
        return false;
    }

    if (fade_duration.get() > kBumpMaxFadeTime) APSA_UNLIKELY {
        return false;
    }

    if (time_since_switch.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar alpha_raw =
        time_since_switch.get() / fade_duration.get();

    if (alpha_raw < kBumpAlphaMin) {
        alpha = kBumpAlphaMin;
    } else if (alpha_raw > kBumpAlphaMax) {
        alpha = kBumpAlphaMax;
    } else {
        alpha = alpha_raw;
    }

    if (!std::isfinite(alpha)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// BLEND CONTROL
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bumpless_blend(
    const scalar u_old,
    const scalar u_new,
    const scalar alpha,
    scalar& u_blended
) noexcept {
    if (!std::isfinite(u_old) || !std::isfinite(u_new)) APSA_UNLIKELY {
        return false;
    }

    if (alpha < kBumpAlphaMin || alpha > kBumpAlphaMax) APSA_UNLIKELY {
        return false;
    }

    u_blended = (scalar{1.0f} - alpha) * u_old + alpha * u_new;

    if (!std::isfinite(u_blended)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// INITIALIZE BUMPLESS TRANSFER
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bumpless_initialize(
    const scalar u_previous,
    const Time fade_duration,
    BumplessTransferState& state
) noexcept {
    if (!std::isfinite(u_previous)) APSA_UNLIKELY {
        return false;
    }

    if (fade_duration.get() < kBumpMinFadeTime) APSA_UNLIKELY {
        return false;
    }

    if (fade_duration.get() > kBumpMaxFadeTime) APSA_UNLIKELY {
        return false;
    }

    state.u_old = u_previous;
    state.u_new = u_previous;
    state.u_blended = u_previous;
    state.alpha = kBumpAlphaMin;
    state.time_since_switch = Time{0.0f};
    state.fade_duration = fade_duration;
    state.transfer_active = true;
    state.valid = true;

    return true;
}

// ═══════════════════════════════════════════
// BUMPLESS TRANSFER UPDATE
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bumpless_update(
    const scalar u_new_command,
    const Time dt,
    BumplessTransferState& state
) noexcept {
    if (!state.valid) APSA_UNLIKELY {
        return false;
    }

    if (!std::isfinite(u_new_command)) APSA_UNLIKELY {
        return false;
    }

    if (dt.get() <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    state.u_new = u_new_command;

    if (!state.transfer_active) {
        state.u_blended = u_new_command;
        return true;
    }

    state.time_since_switch = Time{
        state.time_since_switch.get() + dt.get()
    };

    if (!bumpless_alpha(state.time_since_switch,
                        state.fade_duration,
                        state.alpha)) {
        APSA_UNLIKELY { return false; }
    }

    if (!bumpless_blend(state.u_old, state.u_new,
                        state.alpha, state.u_blended)) {
        APSA_UNLIKELY { return false; }
    }

    if (state.alpha >= kBumpAlphaMax) {
        state.transfer_active = false;
    }

    return true;
}

// ═══════════════════════════════════════════
// TRANSFER COMPLETION CHECK
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool bumpless_is_complete(
    const BumplessTransferState& state,
    bool& complete
) noexcept {
    if (!state.valid) APSA_UNLIKELY {
        return false;
    }

    complete = !state.transfer_active;

    return true;
}