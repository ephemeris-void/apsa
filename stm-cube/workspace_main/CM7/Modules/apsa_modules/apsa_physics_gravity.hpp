#pragma once

#include "apsa_physics_config.hpp"
#include "core/apsa_physics_math.hpp"
#include "core/frame_engine.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <cmath>

using SemiMajorAxis = Quantity<0,1,0>;
using OrbitalPeriod = Time;
#include <algorithm>

// ═══════════════════════════════════════════
// APSA PHYSICS — GRAVITY
// Planet-free | Parameter-driven
// DO-178C Level A | No Hardcoded Planet
// ═══════════════════════════════════════════

// ─── Central Body — Universal Planet Descriptor ───


// ─── Inertia Principal — Gravity Gradient Torque ───

struct InertiaPrincipal {
    scalar Ixx{0.0f};
    scalar Iyy{0.0f};
    scalar Izz{0.0f};
};

// ─── Gravity Potential Result ───

struct GravityPotentialResult {
    Energy potential{};
};

// ─── Surface Gravity Result ───

struct SurfaceGravityResult {
    Acceleration gravity{};
};

// ═══════════════════════════════════════════
// MIN RADIUS
// ═══════════════════════════════════════════

constexpr scalar kGravGradMinRadius{1e4};
constexpr scalar kGravGradMinMu{1e-3};
constexpr scalar kMinGravityRadius{1e4};

// ═══════════════════════════════════════════
// TWO-BODY GRAVITY
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool two_body_gravity(
    const CentralBody& body,
    const UnitVec3<Length>& R,
    UnitVec3<Acceleration>& a
) noexcept {
    if (norm_sq(R).get() <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar r_sq = norm_sq(R).get();
    const scalar r = std::sqrt(r_sq);
    const scalar r_qube = r_sq * r;

    const scalar mu = body.mu.get();
    const scalar r_inv = scalar{1.0f} / r_qube;

    a.x() = -(mu * R.x()) * r_inv;
    a.y() = -(mu * R.y()) * r_inv;
    a.z() = -(mu * R.z()) * r_inv;

    return true;
}

// ═══════════════════════════════════════════
// J2 ACCELERATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool j2_acceleration(
    const CentralBody& body,
    const UnitVec3<Length>& R,
    UnitVec3<Acceleration>& a
) noexcept {
    const scalar mu = body.mu.get();
    const scalar j2 = body.j2.get();
    const scalar radius = body.radius.get();

    const scalar r_sq = norm_sq(R).get();
    if (r_sq <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar r = std::sqrt(r_sq);
    const scalar r_inv = scalar{1.0f} / r;
    const scalar r4 = r_sq * r_sq;

    const scalar x = R.x();
    const scalar y = R.y();
    const scalar z = R.z();

    const scalar z_ratio = (z * z) / r_sq;

    const scalar coeff = (scalar{1.5f} * mu * j2 * radius * radius) / r4;

    const scalar f2 = (scalar{5.0f} * z_ratio) - scalar{1.0f};
    const scalar f3 = (scalar{5.0f} * z_ratio) - scalar{3.0f};

    a.x() = coeff * (x * r_inv) * f2;
    a.y() = coeff * (y * r_inv) * f2;
    a.z() = coeff * (z * r_inv) * f3;

    return true;
}

// ═══════════════════════════════════════════
// J2 SECULAR RATES
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool j2_secular_rates(
    const CentralBody& body,
    const SemiMajorAxis& a,
    const Dimensionless& e,
    const Radian& i,
    Radian& raan_dot,
    Radian& arg_peri_dot,
    OrbitalPeriod& n_corrected
) noexcept {
    const scalar mu = body.mu.get();
    const scalar j2 = body.j2.get();
    const scalar re = body.radius.get();

    const scalar a_val = a.get();
    const scalar e_val = e.get();
    const scalar i_val = i.get();

    if (a_val <= kNormSqThreshold) APSA_UNLIKELY { return false; }
    if (e_val < scalar{0.0f} || e_val >= scalar{1.0f}) APSA_UNLIKELY { return false; }

    const scalar p = a_val * (scalar{1.0f} - e_val * e_val);
    if (p <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar n0 = std::sqrt(mu / (a_val * a_val * a_val));
    const scalar re2 = re * re;
    const scalar p2 = p * p;

    const scalar cos_i = std::cos(i_val);
    const scalar sin_i = std::sin(i_val);

    const scalar factor = (scalar{3.0f} * n0 * j2 * re2) / p2;

    raan_dot = Radian{-scalar{0.5f} * factor * cos_i};
    arg_peri_dot = Radian{scalar{0.25f} * factor * (scalar{5.0f} * cos_i * cos_i - scalar{1.0f})};

    const scalar sqrt_1_e2 = std::sqrt(scalar{1.0f} - e_val * e_val);
    const scalar n_j2_val = n0 * (scalar{1.0f} +
        scalar{0.5f} * factor * (scalar{1.0f} - scalar{1.5f} * sin_i * sin_i) * sqrt_1_e2 / n0);

    n_corrected = OrbitalPeriod{n_j2_val};

    return true;
}

// ═══════════════════════════════════════════
// J3 ACCELERATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool j3_acceleration(
    const UnitVec3<Length>& R,
    const CentralBody& body,
    UnitVec3<Acceleration>& a_j3
) noexcept {
    const scalar r_sq = norm_sq(R).get();
    if (r_sq <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar r = std::sqrt(r_sq);
    const scalar mu = body.mu.get();
    const scalar j3 = body.j3.get();
    const scalar re = body.radius.get();

    const scalar x = R.x();
    const scalar y = R.y();
    const scalar z = R.z();

    const scalar r5 = r_sq * r_sq * r;
    const scalar r7 = r5 * r_sq;
    const scalar z_r = z / r;
    const scalar z2_r2 = z_r * z_r;
    const scalar z3_r3 = z2_r2 * z_r;

    const scalar coeff = mu * j3 * re * re * re;

    const scalar common_xy = coeff * scalar{2.5f} / r7 *
        (scalar{7.0f} * z3_r3 - scalar{3.0f} * z_r);

    const scalar z_term = coeff / r5 *
        (scalar{1.5f} * (scalar{1.0f} - scalar{5.0f} * z2_r2) +
         scalar{17.5f} * z2_r2 * z2_r2 -
         scalar{7.5f} * z2_r2);

    a_j3.x() = x * common_xy;
    a_j3.y() = y * common_xy;
    a_j3.z() = z_term;

    return true;
}

// ═══════════════════════════════════════════
// J4 ACCELERATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool j4_acceleration(
    const UnitVec3<Length>& R,
    const CentralBody& body,
    UnitVec3<Acceleration>& a_j4
) noexcept {
    const scalar r_sq = norm_sq(R).get();
    if (r_sq <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar r = std::sqrt(r_sq);
    const scalar mu = body.mu.get();
    const scalar j4 = body.j4.get();
    const scalar re = body.radius.get();

    const scalar x = R.x();
    const scalar y = R.y();
    const scalar z = R.z();

    const scalar r6 = r_sq * r_sq * r_sq;
    const scalar r8 = r6 * r_sq;

    const scalar z_r = z / r;
    const scalar z2_r2 = z_r * z_r;
    const scalar z4_r4 = z2_r2 * z2_r2;

    const scalar coeff = mu * j4 * re * re * re * re;

    const scalar common_xy = coeff * scalar{1.875f} / r8 *
        (scalar{1.0f} - scalar{14.0f} * z2_r2 + scalar{21.0f} * z4_r4);

    const scalar z_term = coeff / r6 *
        (scalar{1.5f} - scalar{22.5f} * z2_r2 +
         scalar{26.25f} * z4_r4);

    a_j4.x() = x * common_xy;
    a_j4.y() = y * common_xy;
    a_j4.z() = z_term;

    return true;
}

// ═══════════════════════════════════════════
// J3 + J4 HARMONICS
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool j3_j4_harmonics(
    const UnitVec3<Length>& r,
    const CentralBody& body,
    UnitVec3<Acceleration>& a_total
) noexcept {
    UnitVec3<Acceleration> a_j3{};
    UnitVec3<Acceleration> a_j4{};

    if (!j3_acceleration(r, body, a_j3)) APSA_UNLIKELY { return false; }
    if (!j4_acceleration(r, body, a_j4)) APSA_UNLIKELY { return false; }

    a_total = UnitVec3<Acceleration>{a_j3.data + a_j4.data};
    return true;
}

// ═══════════════════════════════════════════
// GRAVITY GRADIENT TORQUE
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool gravity_gradient_torque(
    const UnitVec3<Length>& r,
    const UnitVec3<Dimensionless>& r_hat_body,
    const InertiaPrincipal& inertia,
    const CentralBody& body,
    UnitVec3<Torque>& torque
) noexcept {
    const scalar r_mag = norm(r).get();
    if (r_mag <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar mu = body.mu.get();
    const scalar factor = scalar{3.0f} * mu / (r_mag * r_mag * r_mag);

    const scalar r_bx = r_hat_body.x();
    const scalar r_by = r_hat_body.y();
    const scalar r_bz = r_hat_body.z();

    const scalar tx = factor * (inertia.Izz - inertia.Iyy) * r_by * r_bz;
    const scalar ty = factor * (inertia.Ixx - inertia.Izz) * r_bx * r_bz;
    const scalar tz = factor * (inertia.Iyy - inertia.Ixx) * r_bx * r_by;

    torque = UnitVec3<Torque>{Vec3{tx, ty, tz}};
    return true;
}

// ═══════════════════════════════════════════
// GRAVITY POTENTIAL
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool gravity_potential(
    const UnitVec3<Length>& r,
    const CentralBody& body,
    GravityPotentialResult& result
) noexcept {
    const scalar r_mag = norm(r).get();
    if (r_mag <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar mu = body.mu.get();
    const scalar re = body.radius.get();
    const scalar j2 = body.j2.get();
    const scalar j3 = body.j3.get();
    const scalar j4 = body.j4.get();

    const scalar z = r.z();
    const scalar sin_phi = std::clamp(z / r_mag, -1.0f, 1.0f);

    const scalar p2 = legendre_p2(sin_phi);
    const scalar p3 = legendre_p3(sin_phi);
    const scalar p4 = legendre_p4(sin_phi);

    const scalar re_r = re / r_mag;

    const scalar U = -(mu / r_mag) * (
        scalar{1.0f}
        - j2 * re_r * re_r * p2
        - j3 * re_r * re_r * re_r * p3
        - j4 * re_r * re_r * re_r * re_r * p4
    );

    result.potential = Energy{U};
    return true;
}

// ═══════════════════════════════════════════
// SURFACE GRAVITY
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool surface_gravity(
    const Radian latitude,
    const Length altitude,
    const CentralBody& body,
    SurfaceGravityResult& result
) noexcept {
    const scalar mu = body.mu.get();
    const scalar re = body.radius.get();
    const scalar j2 = body.j2.get();
    const scalar j3 = body.j3.get();
    const scalar j4 = body.j4.get();

    const scalar phi = latitude.get();
    const scalar h = altitude.get();

    if (h < scalar{0.0f}) APSA_UNLIKELY { return false; }

    const scalar sin_phi = std::sin(phi);
    const scalar p2 = legendre_p2(sin_phi);
    const scalar p3 = legendre_p3(sin_phi);
    const scalar p4 = legendre_p4(sin_phi);

    const scalar r = re + h;

    const scalar g = (mu / (r * r)) * (
        scalar{1.0f}
        - j2 * scalar{3.0f} * p2
        - j3 * scalar{4.0f} * p3
        - j4 * scalar{5.0f} * p4
    );

    result.gravity = Acceleration{g};
    return true;
}

// ═══════════════════════════════════════════
// GRAVITY GRADIENT MATRIX
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool gravity_gradient_two_body(
    const UnitVec3<Length>& r,
    const scalar mu,
    Mat3& dg_dr
) noexcept {
    if (mu < kGravGradMinMu) APSA_UNLIKELY {
        return false;
    }

    const scalar r_sq = norm_sq(r).get();

    if (r_sq < kGravGradMinRadius * kGravGradMinRadius) APSA_UNLIKELY {
        return false;
    }

    const scalar r_mag = std::sqrt(r_sq);
    const scalar r_cu = r_sq * r_mag;
    const scalar r_5 = r_cu * r_sq;

    if (r_cu < kNormSqThreshold || r_5 < kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }

    const scalar term1 = -mu / r_cu;
    const scalar term2 = scalar{3.0f} * mu / r_5;

    const scalar x = r.x();
    const scalar y = r.y();
    const scalar z = r.z();

    dg_dr = Mat3{{
        term1 + term2 * x * x, term2 * x * y,        term2 * x * z,
        term2 * y * x,        term1 + term2 * y * y, term2 * y * z,
        term2 * z * x,        term2 * z * y,        term1 + term2 * z * z
    }};

    return true;
}

[[nodiscard]] constexpr bool gravity_gradient_full_A(
    const UnitVec3<Length>& r,
    const scalar mu,
    Matrix_Dynamic<6, 6>& A
) noexcept {
    Mat3 dg_dr{};

    if (!gravity_gradient_two_body(r, mu, dg_dr)) APSA_UNLIKELY {
        return false;
    }

    A = Matrix_Dynamic<6, 6>{6, 6};

    for (std::size_t i{0U}; i < 3U; ++i) {
        A.at(i, i + 3U) = scalar{1.0f};
    }

    for (std::size_t i{0U}; i < 3U; ++i) {
        for (std::size_t j{0U}; j < 3U; ++j) {
            A.at(i + 3U, j) = dg_dr.at(i, j);
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// SURFACE GRAVITY J2
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool surface_gravity_j2(
    const CentralBody& body,
    const Radian latitude,
    UnitVec3<Acceleration>& result
) noexcept {
    const scalar mu = body.mu.get();
    const scalar R = body.radius.get();
    const scalar J2 = body.j2.get();
    const scalar phi = latitude.get();

    if (mu <= kNormSqThreshold || R <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }

    const scalar sin_phi = std::sin(phi);
    const scalar sin2 = sin_phi * sin_phi;

    const scalar correction = scalar{1.5f} * J2 *
        (scalar{3.0f} * sin2 - scalar{1.0f});

    const scalar g_mag = (mu / (R * R)) *
        (scalar{1.0f} - correction);

    if (!std::isfinite(g_mag)) APSA_UNLIKELY {
        return false;
    }

    result = UnitVec3<Acceleration>{Vec3{0.0f, 0.0f, -g_mag}};

    return true;
}

// ═══════════════════════════════════════════
// SPHERICAL HARMONIC — FULL ACCELERATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool spherical_harmonic_accel_full(
    const UnitVec3<Length>& position,
    const CentralBody& body,
    const HarmonicCoeffs& coeffs,
    UnitVec3<Acceleration>& acceleration
) noexcept {
    const scalar mu = body.mu.get();
    const scalar R_e = body.radius.get();

    const Vec3 r_vec = position.data;
    const scalar r = norm(position).get();

    if (r <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar x = r_vec.x();
    const scalar y = r_vec.y();
    const scalar z = r_vec.z();

    const scalar r_xy = std::sqrt(x*x + y*y);

    if (r_xy <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar lat = std::atan2(z, r_xy);
    const scalar lon = std::atan2(y, x);

    const scalar sin_phi = std::sin(lat);
    const scalar cos_phi = std::cos(lat);

    const scalar ratio = R_e / r;

    scalar dU_dr{0.0f};
    scalar dU_dlat{0.0f};
    scalar dU_dlon{0.0f};

    scalar ratio_n = ratio;

    for (std::size_t n{0U}; n <= kMaxHarmonicDegree; ++n) {
        scalar sum_m{0.0f};
        scalar sum_m_lat{0.0f};
        scalar sum_m_lon{0.0f};

        scalar ratio_prev = ratio_n;

        for (std::size_t m{0U}; m <= n; ++m) {
            const std::size_t idx = harmonic_index(n, m);

            const scalar C = coeffs.C_nm[idx];
            const scalar S = coeffs.S_nm[idx];

            const scalar cos_mlon = std::cos(static_cast<scalar>(m) * lon);
            const scalar sin_mlon = std::sin(static_cast<scalar>(m) * lon);

            const scalar sectoral = C * cos_mlon + S * sin_mlon;
            const scalar sectoral_deriv = -C * sin_mlon + S * cos_mlon;

            scalar P_nm{0.0f};
            scalar dP_nm{0.0f};

            const bool p_ok = legendre_associated_with_derivative(
                sin_phi, cos_phi, n, m, P_nm, dP_nm
            );
            if (!p_ok) APSA_UNLIKELY { return false; }

            sum_m += P_nm * sectoral;
            sum_m_lat += dP_nm * sectoral;
            sum_m_lon += static_cast<scalar>(m) * P_nm * sectoral_deriv;
        }

        const scalar factor = ratio_n / r;

        dU_dr += -static_cast<scalar>(n + 1U) * factor *
            ratio_prev / ratio * sum_m;
        dU_dlat += ratio_n * sum_m_lat;
        dU_dlon += ratio_n * sum_m_lon;

        ratio_n *= ratio;
    }

    const scalar mu_r = mu / r;

    const scalar a_r = mu_r * dU_dr;

    const scalar a_lat = mu_r / r * dU_dlat;
    const scalar a_lon = mu_r / (r * r_xy) * dU_dlon;

    const scalar lat_unit_x = -sin_phi * std::cos(lon);
    const scalar lat_unit_y = -sin_phi * std::sin(lon);
    const scalar lat_unit_z = cos_phi;

    const scalar lon_unit_x = -std::sin(lon);
    const scalar lon_unit_y = std::cos(lon);
    const scalar lon_unit_z = 0.0f;

    const scalar r_unit_x = x / r;
    const scalar r_unit_y = y / r;
    const scalar r_unit_z = z / r;

    const Vec3 a_total{
        a_r * r_unit_x + a_lat * lat_unit_x + a_lon * lon_unit_x,
        a_r * r_unit_y + a_lat * lat_unit_y + a_lon * lon_unit_y,
        a_r * r_unit_z + a_lat * lat_unit_z + a_lon * lon_unit_z
    };

    if (!std::isfinite(a_total.x()) ||
        !std::isfinite(a_total.y()) ||
        !std::isfinite(a_total.z())) {
        APSA_UNLIKELY { return false; }
    }

    acceleration = UnitVec3<Acceleration>{a_total};
    return true;
}

// ═══════════════════════════════════════════
// GRAVITY UNCERTAINTY
// ═══════════════════════════════════════════

struct GravitySigmaCoeffs {
    std::array<scalar, kMaxHarmonicCoeffs> sigma_C{};
    std::array<scalar, kMaxHarmonicCoeffs> sigma_S{};
};

[[nodiscard]] constexpr bool gravity_uncertainty(
    const CentralBody& body,
    const UnitVec3<Length>& position,
    const GravitySigmaCoeffs& sigma_coeffs,
    const std::size_t max_degree,
    Acceleration& uncertainty
) noexcept {
    const scalar mu = body.mu.get();
    const scalar R = body.radius.get();
    const scalar r = norm(position).get();

    if (mu <= kNormSqThreshold || R <= kNormSqThreshold ||
        r <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }
    if (max_degree == 0U || max_degree > kMaxHarmonicDegree) {
        APSA_UNLIKELY { return false; }
    }

    const scalar ratio = R / r;
    scalar sum{0.0f};
    scalar compensation{0.0f};

    scalar ratio_2n4 = ratio * ratio * ratio * ratio;

    for (std::size_t n{0U}; n <= max_degree; ++n) {
        scalar sum_m{0.0f};

        for (std::size_t m{0U}; m <= n; ++m) {
            const std::size_t idx = harmonic_index(n, m);

            const scalar sig_C2 = sigma_coeffs.sigma_C[idx] *
                sigma_coeffs.sigma_C[idx];
            const scalar sig_S2 = sigma_coeffs.sigma_S[idx] *
                sigma_coeffs.sigma_S[idx];

            sum_m += sig_C2 + sig_S2;
        }

        const scalar term = static_cast<scalar>((n + 1U) * (n + 1U)) *
            ratio_2n4 * sum_m;

        const scalar y = term - compensation;
        const scalar t = sum + y;
        compensation = (t - sum) - y;
        sum = t;

        ratio_2n4 *= ratio * ratio;
    }

    const scalar sigma = (mu / (R * R)) * std::sqrt(sum);

    if (!std::isfinite(sigma)) APSA_UNLIKELY {
        return false;
    }

    uncertainty = Acceleration{sigma};
    return true;
}