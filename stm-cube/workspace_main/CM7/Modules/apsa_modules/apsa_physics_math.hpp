#pragma once

#include "apsa_physics_config.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>
#include <algorithm>

// ═══════════════════════════════════════════
// APSA PHYSICS — UNIVERSAL MATH
// Planet-free | Vehicle-free | Domain-free
// DO-178C Level A | No Magic Number
// ═══════════════════════════════════════════

// ─── Universal Physics Constants ───

constexpr scalar kMinPhysicalTemp{100.0f};
constexpr scalar kMaxLogValue{700.0f};
constexpr scalar kMinLogValue{-700.0f};

// ─── Universal Max Points ───

constexpr std::size_t kMaxTimePoints{512U};
constexpr std::size_t kMaxIntegrationPoints{512U};
constexpr std::size_t kMaxTablePoints{64U};

// ─── Universal Gravity Min ───

constexpr scalar kGravitationalParameterMin{1e-3};
constexpr scalar kUniversalGasConstant{8.314462618f};
constexpr scalar kSqrt2{1.4142135623730951f};
constexpr scalar kBoltzmann{1.380649e-23};
constexpr scalar kStefanBoltzmann{5.6704e-8};
constexpr std::size_t kMaxHarmonicDegree{70};
constexpr std::size_t kMaxHarmonicCoeffs{5040};

// ═══════════════════════════════════════════
// LEGENDRE POLYNOMIALS
// ═══════════════════════════════════════════

[[nodiscard]] constexpr scalar legendre_p2(
    const scalar x
) noexcept {
    return scalar{0.5f} * (scalar{3.0f} * x * x - scalar{1.0f});
}

[[nodiscard]] constexpr scalar legendre_p3(
    const scalar x
) noexcept {
    return scalar{0.5f} * (scalar{5.0f} * x * x * x - scalar{3.0f} * x);
}

[[nodiscard]] constexpr scalar legendre_p4(
    const scalar x
) noexcept {
    return scalar{0.125f} * (scalar{35.0f} * x * x * x * x -
                            scalar{30.0f} * x * x + scalar{3.0f});
}

// ═══════════════════════════════════════════
// ASSOCIATED LEGENDRE + HARMONIC INDEX
// ═══════════════════════════════════════════


struct HarmonicCoeffs {
    std::array<scalar, kMaxHarmonicCoeffs> C_nm{};
    std::array<scalar, kMaxHarmonicCoeffs> S_nm{};
};

[[nodiscard]] constexpr std::size_t harmonic_index(
    const std::size_t n,
    const std::size_t m
) noexcept {
    return (n * (n + 1U)) / 2U + m;
}

[[nodiscard]] constexpr bool legendre_associated(
    const scalar x,
    const std::size_t n,
    const std::size_t m,
    scalar& result
) noexcept {
    if (m > n) APSA_UNLIKELY {
        return false;
    }

    scalar p_mm = scalar{1.0f};

    if (m > 0U) {
        const scalar sqrt_1_x2 = std::sqrt(
            std::max(scalar{0.0f}, scalar{1.0f} - x * x)
        );
        scalar fact = scalar{1.0f};

        for (std::size_t i{1U}; i <= m; ++i) {
            p_mm *= -fact * sqrt_1_x2;
            fact += scalar{2.0f};
        }
    }

    if (n == m) {
        result = p_mm;
        return true;
    }

    scalar p_mm1 = x * static_cast<scalar>(2U * m + 1U) * p_mm;

    if (n == m + 1U) {
        result = p_mm1;
        return true;
    }

    scalar p_nm = p_mm1;

    for (std::size_t l = m + 2U; l <= n; ++l) {
        const scalar term1 = (scalar{2.0f} * static_cast<scalar>(l) - scalar{1.0f}) *
            x * p_mm1;
        const scalar term2 = static_cast<scalar>(l + m - 1U) * p_mm;
        p_nm = (term1 - term2) / static_cast<scalar>(l - m);

        p_mm = p_mm1;
        p_mm1 = p_nm;
    }

    result = p_nm;
    return true;
}

[[nodiscard]] constexpr bool legendre_associated_with_derivative(
    const scalar x,
    const scalar cos_theta,
    const std::size_t n,
    const std::size_t m,
    scalar& P_nm,
    scalar& dP_nm
) noexcept {
    if (m > n) APSA_UNLIKELY {
        return false;
    }

    scalar P_mm = scalar{1.0f};

    if (m > 0U) {
        const scalar sqrt_1_x2 = std::sqrt(
            std::max(scalar{0.0f}, scalar{1.0f} - x * x)
        );
        scalar fact = scalar{1.0f};

        for (std::size_t i{1U}; i <= m; ++i) {
            P_mm *= -fact * sqrt_1_x2;
            fact += scalar{2.0f};
        }
    }

    if (n == m) {
        P_nm = P_mm;
        dP_nm = static_cast<scalar>(m) * x / cos_theta * P_mm;
        return true;
    }

    scalar P_mm1 = x * static_cast<scalar>(2U * m + 1U) * P_mm;

    if (n == m + 1U) {
        P_nm = P_mm1;
        dP_nm = static_cast<scalar>(n) * x / cos_theta * P_nm -
            static_cast<scalar>(n + m) / cos_theta * P_mm;
        return true;
    }

    scalar P_nm_val = P_mm1;
    scalar P_prev = P_mm;

    for (std::size_t l = m + 2U; l <= n; ++l) {
        const scalar term1 = (scalar{2.0f} * static_cast<scalar>(l) - scalar{1.0f}) *
            x * P_mm1;
        const scalar term2 = static_cast<scalar>(l + m - 1U) * P_mm;
        P_nm_val = (term1 - term2) / static_cast<scalar>(l - m);

        P_mm = P_mm1;
        P_mm1 = P_nm_val;
    }

    P_nm = P_nm_val;

    if (std::abs(cos_theta) <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    dP_nm = (static_cast<scalar>(n) * x * P_nm -
        static_cast<scalar>(n + m) * P_prev) / cos_theta;

    return true;
}

// ═══════════════════════════════════════════
// PARK RATE — Arrhenius
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool park_rate(
    const scalar A_f,
    const scalar n_f,
    const scalar E_a,
    const Temperature control_temp,
    scalar& k_f
) noexcept {
    const scalar T = control_temp.get();

    if (T <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (A_f <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (E_a < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar log_k = std::log(A_f) +
        n_f * std::log(T) -
        E_a / (kUniversalGasConstant * T);

    k_f = std::exp(log_k);

    if (!std::isfinite(k_f)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// STABILITY DERIVATIVE — Finite Difference
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool stability_derivative(
    const std::array<Dimensionless, kMaxTablePoints>& coeff_values,
    const std::array<Radian, kMaxTablePoints>& angle_values,
    const std::size_t count,
    const std::size_t eval_index,
    scalar& derivative
) noexcept {
    if (count < 3U || count > kMaxTablePoints) APSA_UNLIKELY {
        return false;
    }
    if (eval_index >= count) APSA_UNLIKELY {
        return false;
    }

    if (eval_index == 0U) {
        const scalar d_angle = angle_values[1U].get() -
            angle_values[0].get();
        if (std::abs(d_angle) <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }
        derivative = (coeff_values[1U].get() - coeff_values[0].get()) /
            d_angle;
    } else if (eval_index == count - 1U) {
        const scalar d_angle = angle_values[count - 1U].get() -
            angle_values[count - 2U].get();
        if (std::abs(d_angle) <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }
        derivative = (coeff_values[count - 1U].get() -
            coeff_values[count - 2U].get()) / d_angle;
    } else {
        const scalar d_angle = angle_values[eval_index + 1U].get() -
            angle_values[eval_index - 1U].get();
        if (std::abs(d_angle) <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }
        derivative = (coeff_values[eval_index + 1U].get() -
            coeff_values[eval_index - 1U].get()) / d_angle;
    }

    if (!std::isfinite(derivative)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// CENTER OF PRESSURE + STATIC MARGIN
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool center_of_pressure(
    const Length reference_position,
    const Torque pitch_moment,
    const Force normal_force,
    Length& cp_position
) noexcept {
    const scalar x_ref = reference_position.get();
    const scalar M_pitch = pitch_moment.get();
    const scalar F_normal = normal_force.get();

    if (std::abs(F_normal) <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar x_cp = x_ref - M_pitch / F_normal;

    if (!std::isfinite(x_cp)) APSA_UNLIKELY {
        return false;
    }

    cp_position = Length{x_cp};
    return true;
}

[[nodiscard]] constexpr bool static_margin(
    const Length cg_position,
    const Length cp_position,
    const Length reference_length,
    Dimensionless& margin
) noexcept {
    const scalar x_cg = cg_position.get();
    const scalar x_cp = cp_position.get();
    const scalar L_ref = reference_length.get();

    if (L_ref <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar K_n = (x_cg - x_cp) / L_ref;

    if (!std::isfinite(K_n)) APSA_UNLIKELY {
        return false;
    }

    margin = Dimensionless{K_n};
    return true;
}

// ═══════════════════════════════════════════
// KNUDSEN NUMBER — Rarefied Gas
// ═══════════════════════════════════════════

constexpr scalar kCO2Diameter{0.33e-9};

[[nodiscard]] constexpr bool knudsen_number(
    const Temperature temperature,
    const Pressure pressure,
    const Length reference_length,
    Dimensionless& kn
) noexcept {
    const scalar T = temperature.get();
    const scalar p = pressure.get();
    const scalar L = reference_length.get();

    if (T <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (p <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (L <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar d2 = kCO2Diameter * kCO2Diameter;
    const scalar denom = kSqrt2 * kPi * d2 * p * L;

    const scalar Kn = (kBoltzmann * T) / denom;

    if (!std::isfinite(Kn)) APSA_UNLIKELY {
        return false;
    }

    kn = Dimensionless{Kn};
    return true;
}