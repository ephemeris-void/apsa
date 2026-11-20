#pragma once

#include "apsa_estimation_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// ═══════════════════════════════════════════
// APSA ESTIMATION — INNOVATION MONITOR
// NIS/NEES + IMU Health + Skip Logic
// Universal — no domain-specific
// DO-178C Level A | No Hardcoded Vehicle
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// NIS/NEES DIMENSIONS
// ═══════════════════════════════════════════

constexpr std::size_t kNisMaxMeasurement{9};
constexpr std::size_t kNisMaxState{21};

// ═══════════════════════════════════════════
// CHI-SQUARE CRITICAL VALUES
// ═══════════════════════════════════════════

constexpr std::array<scalar, 9> kChiSquare95{{
    3.841f,    // m=1
    5.991f,    // m=2
    7.815f,    // m=3
    9.488f,    // m=4
    11.070f,   // m=5
    12.592f,   // m=6
    14.067f,   // m=7
    15.507f,   // m=8
    16.919f    // m=9
}};

constexpr std::array<scalar, 9> kChiSquare99_73{{
    9.000f,    // m=1
    11.829f,   // m=2
    14.156f,   // m=3
    16.251f,   // m=4
    18.206f,   // m=5
    20.062f,   // m=6
    21.850f,   // m=7
    23.574f,   // m=8
    25.256f    // m=9
}};

constexpr std::array<scalar, 9> kChiSquare99_99{{
    15.137f,   // m=1
    18.421f,   // m=2
    21.108f,   // m=3
    23.512f,   // m=4
    25.745f,   // m=5
    27.856f,   // m=6
    29.877f,   // m=7
    31.828f,   // m=8
    33.720f    // m=9
}};

// ═══════════════════════════════════════════
// IMU AXIS CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kImuAxisCount{6};
constexpr scalar kImuAxisChiSquareThreshold{15.137f};
constexpr scalar kImuAxisWarningThreshold{6.635f};

// ═══════════════════════════════════════════
// IMU HEALTH CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kImuHealthMaxAxes{6};
constexpr scalar kImuHealthExpectedNis{1.0f};
constexpr scalar kImuHealthCriticalNis{15.137f};
constexpr scalar kImuHealthMinimum{0.0f};
constexpr scalar kImuHealthMaximum{1.0f};

// ═══════════════════════════════════════════
// SKIP LOGIC CONSTANTS
// ═══════════════════════════════════════════

constexpr std::size_t kSkipMaxMeasurement{9};
constexpr std::size_t kSkipMaxConsecutive{10};
constexpr scalar kSkipSigmaThreshold{3.0f};
constexpr scalar kSkipRecoveryThreshold{1.5f};

// ═══════════════════════════════════════════
// FILTER HEALTH ENUM
// ═══════════════════════════════════════════

enum class FilterHealth : std::uint8_t {
    NOMINAL = 0U,
    DEGRADED = 1U,
    FAULT = 2U
};

// ═══════════════════════════════════════════
// NIS RESULT
// ═══════════════════════════════════════════

struct NisResult {
    scalar nis_value{0.0f};
    scalar gate_threshold{0.0f};
    bool gate_passed{true};
    FilterHealth health{FilterHealth::NOMINAL};
};

// ═══════════════════════════════════════════
// NEES RESULT
// ═══════════════════════════════════════════

struct NeesResult {
    scalar nees_value{0.0f};
    scalar lower_bound{0.0f};
    scalar upper_bound{0.0f};
    bool consistent{true};
};

// ═══════════════════════════════════════════
// NIS CALCULATION — Cholesky Solve
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool nis_calculate(
    const Vector_Dynamic<kNisMaxMeasurement>& innovation,
    const Matrix_Dynamic<kNisMaxMeasurement, kNisMaxMeasurement>& S,
    const scalar gate_confidence,
    NisResult& result
) noexcept {
    const std::size_t m = innovation.size();

    if (m == 0U || m > kNisMaxMeasurement) APSA_UNLIKELY {
        return false;
    }

    if (S.rows() != m || S.cols() != m) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(S)) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kNisMaxMeasurement, kNisMaxMeasurement> L{m, m};

    if (!cholesky(S, L)) APSA_UNLIKELY {
        return false;
    }

    Vector_Dynamic<kNisMaxMeasurement> y{m};

    if (!forward_substitute(innovation, L, y)) APSA_UNLIKELY {
        return false;
    }

    scalar nis{0.0f};

    for (std::size_t i{0U}; i < m; ++i) {
        nis += y.at(i) * y.at(i);
    }

    if (!std::isfinite(nis)) APSA_UNLIKELY {
        return false;
    }

    if (nis < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    scalar threshold{0.0f};

    if (m <= kChiSquare99_99.size()) {
        threshold = kChiSquare99_99[m - 1U];
    } else {
        threshold = scalar{50.0f};
    }

    result.nis_value = nis;
    result.gate_threshold = threshold;
    result.gate_passed = (nis <= threshold);

    if (!result.gate_passed) {
        result.health = FilterHealth::FAULT;
    } else if (nis > kChiSquare95[m - 1U]) {
        result.health = FilterHealth::DEGRADED;
    } else {
        result.health = FilterHealth::NOMINAL;
    }

    return true;
}

// ═══════════════════════════════════════════
// NEES CALCULATION — Cholesky Solve
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool nees_calculate(
    const Vector_Dynamic<kNisMaxState>& estimation_error,
    const Matrix_Dynamic<kNisMaxState, kNisMaxState>& P,
    const std::size_t window_size,
    NeesResult& result
) noexcept {
    const std::size_t n = estimation_error.size();

    if (n == 0U || n > kNisMaxState) APSA_UNLIKELY {
        return false;
    }

    if (P.rows() != n || P.cols() != n) APSA_UNLIKELY {
        return false;
    }

    if (!is_positive_definite(P)) APSA_UNLIKELY {
        return false;
    }

    if (window_size == 0U) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<kNisMaxState, kNisMaxState> L{n, n};

    if (!cholesky(P, L)) APSA_UNLIKELY {
        return false;
    }

    Vector_Dynamic<kNisMaxState> y{n};

    if (!forward_substitute(estimation_error, L, y)) APSA_UNLIKELY {
        return false;
    }

    scalar nees{0.0f};

    for (std::size_t i{0U}; i < n; ++i) {
        nees += y.at(i) * y.at(i);
    }

    if (!std::isfinite(nees)) APSA_UNLIKELY {
        return false;
    }

    const scalar dof = static_cast<scalar>(n * window_size);

    const scalar lower = dof - scalar{2.0f} * std::sqrt(scalar{2.0f} * dof);
    const scalar upper = dof + scalar{2.0f} * std::sqrt(scalar{2.0f} * dof);

    result.nees_value = nees;
    result.lower_bound = lower;
    result.upper_bound = upper;
    result.consistent = (nees >= lower) && (nees <= upper);

    return true;
}

// ═══════════════════════════════════════════
// NIS ACCUMULATOR — Time-Averaged
// ═══════════════════════════════════════════

class NisAccumulator {
private:
    scalar sum_{0.0f};
    scalar sum_sq_{0.0f};
    std::size_t count_{0U};

public:
    constexpr NisAccumulator() noexcept = default;

    [[nodiscard]] constexpr bool add(
        const scalar nis_value
    ) noexcept {
        if (!std::isfinite(nis_value)) APSA_UNLIKELY {
            return false;
        }

        if (nis_value < scalar{0.0f}) APSA_UNLIKELY {
            return false;
        }

        if (count_ == static_cast<std::size_t>(-1)) APSA_UNLIKELY {
            return false;
        }

        sum_ += nis_value;
        sum_sq_ += nis_value * nis_value;
        ++count_;

        return true;
    }

    [[nodiscard]] constexpr bool get_statistics(
        scalar& mean,
        scalar& variance
    ) const noexcept {
        if (count_ == 0U) APSA_UNLIKELY {
            return false;
        }

        const scalar inv_count = scalar{1.0f} / static_cast<scalar>(count_);

        mean = sum_ * inv_count;

        const scalar mean_sq = sum_sq_ * inv_count;
        variance = mean_sq - mean * mean;

        if (variance < scalar{0.0f}) {
            variance = scalar{0.0f};
        }

        return true;
    }

    [[nodiscard]] constexpr std::size_t count() const noexcept {
        return count_;
    }

    constexpr void reset() noexcept {
        sum_ = scalar{0.0f};
        sum_sq_ = scalar{0.0f};
        count_ = 0U;
    }
};

// ═══════════════════════════════════════════
// CHI-SQUARE GATE
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool chi_square_gate(
    const scalar nis_value,
    const std::size_t measurement_dim,
    const scalar confidence_level,
    bool& gate_passed
) noexcept {
    if (measurement_dim == 0U ||
        measurement_dim > kChiSquare99_99.size()) APSA_UNLIKELY {
        return false;
    }

    if (nis_value < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    scalar threshold{0.0f};

    if (confidence_level >= scalar{0.9999f}) {
        threshold = kChiSquare99_99[measurement_dim - 1U];
    } else if (confidence_level >= scalar{0.9973f}) {
        threshold = kChiSquare99_73[measurement_dim - 1U];
    } else {
        threshold = kChiSquare95[measurement_dim - 1U];
    }

    gate_passed = (nis_value <= threshold);

    return true;
}

// ═══════════════════════════════════════════
// IMU AXIS ENUM
// ═══════════════════════════════════════════

enum class ImuAxis : std::uint8_t {
    GYRO_X = 0U,
    GYRO_Y = 1U,
    GYRO_Z = 2U,
    ACCEL_X = 3U,
    ACCEL_Y = 4U,
    ACCEL_Z = 5U
};

// ═══════════════════════════════════════════
// PER-AXIS NIS STATE
// ═══════════════════════════════════════════

struct ImuAxisNisState {
    std::array<scalar, kImuAxisCount> nis_values{};
    std::array<scalar, kImuAxisCount> innovation{};
    std::array<scalar, kImuAxisCount> S_diagonal{};
    std::array<bool, kImuAxisCount> fault_flags{};
    std::array<bool, kImuAxisCount> warning_flags{};
    bool initialized{false};
};

// ═══════════════════════════════════════════
// PER-AXIS NIS CALCULATION
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_axis_nis_calculate(
    const std::array<scalar, kImuAxisCount>& innovation,
    const std::array<scalar, kImuAxisCount>& S_diagonal,
    ImuAxisNisState& state
) noexcept {
    for (std::size_t i{0U}; i < kImuAxisCount; ++i) {
        if (S_diagonal[i] <= scalar{0.0f}) APSA_UNLIKELY {
            return false;
        }

        state.innovation[i] = innovation[i];
        state.S_diagonal[i] = S_diagonal[i];

        const scalar nis_i =
            innovation[i] * innovation[i] / S_diagonal[i];

        if (!std::isfinite(nis_i)) APSA_UNLIKELY {
            return false;
        }

        if (nis_i < scalar{0.0f}) APSA_UNLIKELY {
            return false;
        }

        state.nis_values[i] = nis_i;

        state.fault_flags[i] =
            (nis_i > kImuAxisChiSquareThreshold);

        state.warning_flags[i] =
            (nis_i > kImuAxisWarningThreshold) && !state.fault_flags[i];
    }

    state.initialized = true;
    return true;
}

// ═══════════════════════════════════════════
// AXIS ISOLATION — Identify Faulty Axis
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_axis_isolate_fault(
    const ImuAxisNisState& state,
    std::array<ImuAxis, kImuAxisCount>& faulty_axes,
    std::size_t& faulty_count
) noexcept {
    if (!state.initialized) APSA_UNLIKELY {
        return false;
    }

    faulty_count = 0U;

    for (std::size_t i{0U}; i < kImuAxisCount; ++i) {
        if (state.fault_flags[i]) {
            faulty_axes[faulty_count] = static_cast<ImuAxis>(i);
            ++faulty_count;
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// IMU HEALTH METRIC — Worst-Axis Based
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_axis_health_metric(
    const ImuAxisNisState& state,
    scalar& health
) noexcept {
    if (!state.initialized) APSA_UNLIKELY {
        return false;
    }

    scalar worst_nis{0.0f};

    for (std::size_t i{0U}; i < kImuAxisCount; ++i) {
        if (state.nis_values[i] > worst_nis) {
            worst_nis = state.nis_values[i];
        }
    }

    constexpr scalar kExpectedNis{1.0f};
    constexpr scalar kHealthScale{10.0f};

    const scalar excess = std::max(scalar{0.0f}, worst_nis - kExpectedNis);

    health = std::exp(-excess / kHealthScale);

    if (!std::isfinite(health)) APSA_UNLIKELY {
        return false;
    }

    if (health < scalar{0.0f} || health > scalar{1.0f}) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// SKIP DECISION — Reject Outlier
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_axis_skip_decision(
    const ImuAxisNisState& state,
    const std::size_t max_faulty_axes,
    bool& skip_update
) noexcept {
    if (!state.initialized) APSA_UNLIKELY {
        return false;
    }

    std::size_t fault_count{0U};

    for (std::size_t i{0U}; i < kImuAxisCount; ++i) {
        if (state.fault_flags[i]) {
            ++fault_count;
        }
    }

    skip_update = (fault_count > max_faulty_axes);

    return true;
}

// ═══════════════════════════════════════════
// RESET
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_axis_nis_reset(
    ImuAxisNisState& state
) noexcept {
    for (auto& v : state.nis_values) {
        v = scalar{0.0f};
    }

    for (auto& v : state.innovation) {
        v = scalar{0.0f};
    }

    for (auto& v : state.S_diagonal) {
        v = scalar{0.0f};
    }

    for (auto& f : state.fault_flags) {
        f = false;
    }

    for (auto& w : state.warning_flags) {
        w = false;
    }

    state.initialized = false;

    return true;
}

// ═══════════════════════════════════════════
// IMU HEALTH STATE
// ═══════════════════════════════════════════

struct ImuHealthState {
    std::array<scalar, kImuHealthMaxAxes> axis_health{};
    scalar overall_health{1.0f};
    bool initialized{false};
};

// ═══════════════════════════════════════════
// SINGLE AXIS HEALTH — Exponential Decay
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_health_single_axis(
    const scalar nis_value,
    scalar& health
) noexcept {
    if (!std::isfinite(nis_value)) APSA_UNLIKELY {
        return false;
    }

    if (nis_value < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar denom = kImuHealthCriticalNis - kImuHealthExpectedNis;

    if (denom <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar excess = nis_value - kImuHealthExpectedNis;

    if (excess <= scalar{0.0f}) {
        health = kImuHealthMaximum;
        return true;
    }

    health = std::exp(-excess / denom);

    if (health < kImuHealthMinimum) {
        health = kImuHealthMinimum;
    }

    if (health > kImuHealthMaximum) {
        health = kImuHealthMaximum;
    }

    if (!std::isfinite(health)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// ALL AXES HEALTH — Overall Metric
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_health_all_axes(
    const std::array<scalar, kImuHealthMaxAxes>& nis_values,
    const std::size_t axis_count,
    ImuHealthState& state
) noexcept {
    if (axis_count == 0U || axis_count > kImuHealthMaxAxes) {
        APSA_UNLIKELY { return false; }
    }

    scalar worst_health = kImuHealthMaximum;

    for (std::size_t i{0U}; i < axis_count; ++i) {
        scalar axis_h{0.0f};

        if (!imu_health_single_axis(nis_values[i], axis_h)) {
            APSA_UNLIKELY { return false; }
        }

        state.axis_health[i] = axis_h;

        if (axis_h < worst_health) {
            worst_health = axis_h;
        }
    }

    state.overall_health = worst_health;

    if (!std::isfinite(state.overall_health)) APSA_UNLIKELY {
        return false;
    }

    state.initialized = true;
    return true;
}

// ═══════════════════════════════════════════
// HEALTH STATUS CLASSIFICATION
// ═══════════════════════════════════════════

enum class ImuHealthStatus : std::uint8_t {
    NOMINAL = 0U,
    DEGRADED = 1U,
    CRITICAL = 2U,
    FAILED = 3U
};

[[nodiscard]] inline constexpr bool imu_health_classify(
    const scalar overall_health,
    ImuHealthStatus& status
) noexcept {
    if (!std::isfinite(overall_health)) APSA_UNLIKELY {
        return false;
    }

    constexpr scalar kDegradedThreshold{0.8f};
    constexpr scalar kCriticalThreshold{0.5f};
    constexpr scalar kFailedThreshold{0.2f};

    if (overall_health >= kDegradedThreshold) {
        status = ImuHealthStatus::NOMINAL;
    } else if (overall_health >= kCriticalThreshold) {
        status = ImuHealthStatus::DEGRADED;
    } else if (overall_health >= kFailedThreshold) {
        status = ImuHealthStatus::CRITICAL;
    } else {
        status = ImuHealthStatus::FAILED;
    }

    return true;
}

// ═══════════════════════════════════════════
// TIME-AVERAGED HEALTH SMOOTHING
// ═══════════════════════════════════════════

struct ImuHealthSmoother {
    scalar smoothed_health{1.0f};
    scalar alpha{0.1f};
    bool initialized{false};
};

[[nodiscard]] inline constexpr bool imu_health_smooth(
    const scalar current_health,
    ImuHealthSmoother& smoother
) noexcept {
    if (!std::isfinite(current_health)) APSA_UNLIKELY {
        return false;
    }

    if (smoother.alpha <= scalar{0.0f} ||
        smoother.alpha > scalar{1.0f}) APSA_UNLIKELY {
        return false;
    }

    if (!smoother.initialized) {
        smoother.smoothed_health = current_health;
        smoother.initialized = true;
        return true;
    }

    smoother.smoothed_health =
        smoother.alpha * current_health +
        (scalar{1.0f} - smoother.alpha) * smoother.smoothed_health;

    if (!std::isfinite(smoother.smoothed_health)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// RESET
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool imu_health_reset(
    ImuHealthState& state
) noexcept {
    for (auto& h : state.axis_health) {
        h = scalar{1.0f};
    }

    state.overall_health = scalar{1.0f};
    state.initialized = false;

    return true;
}

// ═══════════════════════════════════════════
// SKIP LOGIC STATE
// ═══════════════════════════════════════════

struct SkipLogicState {
    std::size_t consecutive_skips{0U};
    bool skipped_previous{false};
    bool initialized{false};
};

// ═══════════════════════════════════════════
// SKIP DECISION — Single Measurement
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool skip_decision_single(
    const scalar innovation,
    const scalar innovation_variance,
    bool& skip
) noexcept {
    if (!std::isfinite(innovation)) APSA_UNLIKELY {
        return false;
    }

    if (innovation_variance <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar sigma_nu = std::sqrt(innovation_variance);

    const scalar normalized = std::abs(innovation) / sigma_nu;

    skip = (normalized > kSkipSigmaThreshold);

    if (!std::isfinite(normalized)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// SKIP DECISION — Multi-Measurement
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool skip_decision_multi(
    const Vector_Dynamic<kSkipMaxMeasurement>& innovation,
    const Vector_Dynamic<kSkipMaxMeasurement>& S_diagonal,
    const std::size_t measurement_count,
    bool& skip
) noexcept {
    if (measurement_count == 0U ||
        measurement_count > kSkipMaxMeasurement) APSA_UNLIKELY {
        return false;
    }

    std::size_t outlier_count{0U};

    for (std::size_t i{0U}; i < measurement_count; ++i) {
        if (S_diagonal.at(i) <= scalar{0.0f}) APSA_UNLIKELY {
            return false;
        }

        const scalar sigma = std::sqrt(S_diagonal.at(i));
        const scalar normalized = std::abs(innovation.at(i)) / sigma;

        if (normalized > kSkipSigmaThreshold) {
            ++outlier_count;
        }
    }

    skip = (outlier_count > 0U);

    return true;
}

// ═══════════════════════════════════════════
// SKIP LOGIC UPDATE — Hold State
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool skip_logic_update(
    const bool skip_current,
    SkipLogicState& state
) noexcept {
    if (!state.initialized) {
        state.consecutive_skips = 0U;
        state.skipped_previous = false;
        state.initialized = true;
    }

    if (skip_current) {
        ++state.consecutive_skips;
        state.skipped_previous = true;

        if (state.consecutive_skips > kSkipMaxConsecutive) {
            APSA_UNLIKELY { return false; }
        }
    } else {
        state.consecutive_skips = 0U;
        state.skipped_previous = false;
    }

    return true;
}

// ═══════════════════════════════════════════
// RECOVERY DECISION — Re-accept Measurement
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool skip_recovery_decision(
    const scalar innovation,
    const scalar innovation_variance,
    const SkipLogicState& state,
    bool& recovered
) noexcept {
    if (!state.initialized) APSA_UNLIKELY {
        return false;
    }

    if (innovation_variance <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar sigma = std::sqrt(innovation_variance);
    const scalar normalized = std::abs(innovation) / sigma;

    recovered = (normalized < kSkipRecoveryThreshold) &&
                state.skipped_previous;

    return true;
}

// ═══════════════════════════════════════════
// HOLD STATE — No Update Applied
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool skip_hold_state(
    const Vector_Dynamic<kSkipMaxMeasurement>& x_pred,
    const Matrix_Dynamic<kSkipMaxMeasurement, kSkipMaxMeasurement>& P_pred,
    Vector_Dynamic<kSkipMaxMeasurement>& x_held,
    Matrix_Dynamic<kSkipMaxMeasurement, kSkipMaxMeasurement>& P_held
) noexcept {
    x_held = x_pred;
    P_held = P_pred;

    return true;
}

// ═══════════════════════════════════════════
// RESET
// ═══════════════════════════════════════════

[[nodiscard]] inline constexpr bool skip_logic_reset(
    SkipLogicState& state
) noexcept {
    state.consecutive_skips = 0U;
    state.skipped_previous = false;
    state.initialized = false;

    return true;
}