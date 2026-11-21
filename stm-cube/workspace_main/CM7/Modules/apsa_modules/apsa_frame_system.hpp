#pragma once
#include "apsa_unit_system.hpp"
#include "apsa_math_engine.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string_view>

// ═══════════════════════════════════════════
// FRAME SYSTEM — APSA Flight Software
// Time → Earth Orientation → Frame Transform → Mars Frame
// প্রতিটা block-এর chain নিজের জায়গায়, extreme guard সহ
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// [1] CENTRAL BODY
// Mars/Earth-এর physical parameter
// ═══════════════════════════════════════════

struct CentralBody {

    GravitationalParameter mu{};
    Length radius{};
    Dimensionless flattening{};
    Dimensionless j2{};
    Dimensionless j3{};
    Dimensionless j4{};
    AngularVel rotation_rate{};
};

constexpr scalar kFramePi{3.14159265358979323846f};
constexpr scalar kFrameTwoPi{6.28318530717958647692f};
constexpr scalar kFrameHalfPi{1.57079632679489661923f};

// ─── Earth Constants ───
inline constexpr GravitationalParameter kEarthMu{3.986004418e14};
inline constexpr Length kEarthRadius{6378137.0f};
inline constexpr Dimensionless kEarthFlattening{1.0f / 298.257223563f};
inline constexpr Dimensionless kEarthJ2{1.08262668e-3};
inline constexpr Dimensionless kEarthJ3{-2.5327e-6};
inline constexpr Dimensionless kEarthJ4{-1.6196e-6};
inline constexpr AngularVel kEarthRotationRate{7.2921150e-5};

// ─── Mars Constants ───
inline constexpr GravitationalParameter kMarsMu{4.282837e13};
inline constexpr Length kMarsRadius{3396190.0f};
inline constexpr Dimensionless kMarsFlattening{0.005886008f};
inline constexpr Dimensionless kMarsJ2{1.95637e-3};
inline constexpr Dimensionless kMarsJ3{3.14530e-5};
inline constexpr Dimensionless kMarsJ4{-1.53760e-5};
inline constexpr AngularVel kMarsRotationRate{7.088218127e-5};

// ═══════════════════════════════════════════
// [2] TIME SYSTEM
// UTC → TAI → TT → TDB
// ═══════════════════════════════════════════

inline constexpr Time kTaiMinusUtc{37.0f};
inline constexpr Time kTtMinusTai{32.184f};
inline constexpr Dimensionless kJ2000JD{2451545.0f};
inline constexpr Dimensionless kDaysPerCentury{36525.0f};
inline constexpr Time kSecondsPerDay{86400.0f};
inline constexpr Time kDefaultDeltaAT{37.0f};
inline constexpr Angle kDefaultEOP{0.0f};

struct TimeState {
    Time tai{};
    Time tt{};
    Time tdb{};
    Time ut1{};
    Dimensionless jd_tdb{};
    Dimensionless t_centuries{};
    Dimensionless du_days{};
};

struct EOPData {
    Angle xp{0.0f};
    Angle yp{0.0f};
    Time dUT1{0.0f};
    Angle dPsi{0.0f};
    Angle dEps{0.0f};
    Time delta_AT{0.0f};
};

struct TDBTerm {
    Time amplitude{};
    Dimensionless frequency{};
    Radian phase{};
};

constexpr std::array<TDBTerm, 7> kTDBTerms{{
    {Time{0.001657f}, Dimensionless{628.3076f}, Radian{6.2401f}},
    {Time{0.000022f}, Dimensionless{575.3385f}, Radian{4.2970f}},
    {Time{0.000014f}, Dimensionless{1256.6152f}, Radian{6.1969f}},
    {Time{0.000005f}, Dimensionless{606.9777f}, Radian{4.0212f}},
    {Time{0.000005f}, Dimensionless{52.9691f}, Radian{0.4444f}},
    {Time{0.000002f}, Dimensionless{21.3299f}, Radian{5.5431f}},
    {Time{0.000010f}, Dimensionless{628.3076f}, Radian{4.2490f}}
}};

constexpr Time kMaxValidTime{86400.0f * 365.25f * 20.0f};

[[nodiscard]] constexpr bool tt_tdb(
    const Time& tt,
    const std::array<TDBTerm, 7>& terms,
    Time& tdb
) noexcept {
    if (tt.get() < scalar{0.0f} || tt.get() > kMaxValidTime.get()) APSA_UNLIKELY {
        return false;
    }

    const Dimensionless JD_tt = (tt / kSecondsPerDay) + kJ2000JD;
    const Dimensionless T = (JD_tt - kJ2000JD) / kDaysPerCentury;

    scalar correction{0.0f};

    for (std::size_t i{0}; i < terms.size(); ++i) {
        const TDBTerm& term = terms[i];
        scalar amplitude = term.amplitude.get();
        const scalar frequency = term.frequency.get();
        const scalar phase = term.phase.get();
        const scalar t = T.get();

        const scalar arg = std::fma(frequency, t, phase);

        if (i == 6U) APSA_UNLIKELY {
            amplitude *= t;
        }

        correction = std::fma(amplitude, std::sin(arg), correction);
    }

    tdb = Time{tt.get() + correction};

    if (!std::isfinite(tdb.get())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool tai_tt(const Time tai, Time& tt) noexcept {
    if (tai.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    tt = tai + kTtMinusTai;

    if (!std::isfinite(tt.get())) APSA_UNLIKELY {
        return false;
    }
    return true;
}

[[nodiscard]] constexpr bool utc_tai(const Time utc, const Time delta_AT, Time& tai) noexcept {
    if (utc.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    tai = utc + delta_AT;

    if (!std::isfinite(tai.get())) APSA_UNLIKELY {
        return false;
    }
    return true;
}

[[nodiscard]] constexpr bool utc_ut1(const Time utc, const Time dUT1, Time& ut1) noexcept {
    if (utc.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    ut1 = utc + dUT1;

    if (!std::isfinite(ut1.get())) APSA_UNLIKELY {
        return false;
    }
    return true;
}

inline constexpr Dimensionless kERA0{0.7790572732640f};
inline constexpr Dimensionless kERARate{1.00273781191135448f};

[[nodiscard]] constexpr bool earth_rotation_angle(
    const Time ut1,
    Radian& earth_angle
) noexcept {
    if (ut1.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const Dimensionless JD_ut1 = (ut1 / kSecondsPerDay) + kJ2000JD;
    const Dimensionless Du = JD_ut1 - kJ2000JD;

    const scalar era_raw = kFrameTwoPi *
        (kERA0.get() + kERARate.get() * Du.get());

    const scalar era_mod = std::fmod(era_raw, kFrameTwoPi);
    const scalar era_final = (era_mod < scalar{0.0f})
        ? era_mod + kFrameTwoPi
        : era_mod;

    if (!std::isfinite(era_final)) APSA_UNLIKELY {
        return false;
    }

    earth_angle = Radian{era_final};
    return true;
}

[[nodiscard]] constexpr bool tdb_jd(
    const Time tdb,
    Dimensionless& JD
) noexcept {
    if (tdb.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    JD = (tdb / kSecondsPerDay) + kJ2000JD;

    if (!std::isfinite(JD.get())) APSA_UNLIKELY {
        return false;
    }
    return true;
}

//------------------------------------
//------------------------------------


// ═══════════════════════════════════════════
// [3] ROTATION MATRICES
// r1/r2/r3 — x/y/z অক্ষে rotation
// Extreme: যেকোনো angle, FMA precision ধরে
// ═══════════════════════════════════════════

[[nodiscard]] constexpr Mat3 r1(const Radian theta) noexcept {
    const scalar c = std::cos(theta.get());
    const scalar s = std::sin(theta.get());

    return Mat3{{
        1.0f, 0.0f, 0.0f,
        0.0f, c,   s,
        0.0f, -s,  c
    }};
}

[[nodiscard]] constexpr Mat3 r2(const Radian theta) noexcept {
    const scalar c = std::cos(theta.get());
    const scalar s = std::sin(theta.get());

    return Mat3{{
        c, 0.0f, -s,
        0.0f, 1.0f, 0.0f,
        s, 0.0f, c
    }};
}

[[nodiscard]] constexpr Mat3 r3(const Radian theta) noexcept {
    const scalar c = std::cos(theta.get());
    const scalar s = std::sin(theta.get());

    return Mat3{{
        c,  s, 0.0f,
        -s, c, 0.0f,
        0.0f, 0.0f, 1.0f
    }};
}

// ═══════════════════════════════════════════
// [4] GCRS ↔ ITRS — POSITION + VELOCITY
// Extreme: zero vector guard, finite check
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool gcrs_to_itrs(
    const Mat3& Q,
    const Mat3& R,
    const Mat3& W,
    const UnitVec3<Length>& r_gcrs,
    UnitVec3<Length>& r_itrs
) noexcept {
    if (norm_sq(r_gcrs).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Mat3 WRQ = W * R * Q;
    r_itrs = UnitVec3<Length>{WRQ * r_gcrs.data};

    return std::isfinite(r_itrs.x()) &&
           std::isfinite(r_itrs.y()) &&
           std::isfinite(r_itrs.z());
}

[[nodiscard]] constexpr bool itrs_to_gcrs(
    const Mat3& Q,
    const Mat3& R,
    const Mat3& W,
    const UnitVec3<Length>& r_itrs,
    UnitVec3<Length>& r_gcrs
) noexcept {
    if (norm_sq(r_itrs).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Mat3 WRQ = W * R * Q;
    const Mat3 WRQ_T = transpose(WRQ);
    r_gcrs = UnitVec3<Length>{WRQ_T * r_itrs.data};

    return std::isfinite(r_gcrs.x()) &&
           std::isfinite(r_gcrs.y()) &&
           std::isfinite(r_gcrs.z());
}

[[nodiscard]] constexpr bool gcrs_to_itrs_velocity(
    const Mat3& Q,
    const Mat3& R,
    const Mat3& W,
    const UnitVec3<AngularVel>& earth_rate,
    const UnitVec3<Length>& r_gcrs,
    const UnitVec3<Velocity>& v_gcrs,
    UnitVec3<Velocity>& v_itrs
) noexcept {
    if (norm_sq(r_gcrs).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Mat3 WRQ = W * R * Q;
    const Vec3 v_rot = WRQ * v_gcrs.data;

    const auto cross_term = cross(earth_rate, r_gcrs);
    const Vec3 result = v_rot - cross_term.data;

    v_itrs = UnitVec3<Velocity>{result};

    return std::isfinite(v_itrs.x()) &&
           std::isfinite(v_itrs.y()) &&
           std::isfinite(v_itrs.z());
}

[[nodiscard]] constexpr bool itrs_to_gcrs_velocity(
    const Mat3& Q,
    const Mat3& R,
    const Mat3& W,
    const UnitVec3<AngularVel>& earth_rate,
    const UnitVec3<Length>& r_itrs,
    const UnitVec3<Velocity>& v_itrs,
    UnitVec3<Velocity>& v_gcrs
) noexcept {
    if (norm_sq(r_itrs).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Mat3 WRQ = W * R * Q;
    const Mat3 WRQ_T = transpose(WRQ);

    const auto cross_term = cross(earth_rate, r_itrs);
    const Vec3 v_sum = v_itrs.data + cross_term.data;

    v_gcrs = UnitVec3<Velocity>{WRQ_T * v_sum};

    return std::isfinite(v_gcrs.x()) &&
           std::isfinite(v_gcrs.y()) &&
           std::isfinite(v_gcrs.z());
}

// ═══════════════════════════════════════════
// [5] LVLH ↔ GCRS
// Local orbital frame
// Extreme: zero angular momentum guard
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool lvlh_to_gcrs(
    const UnitVec3<Length>& r_gcrs,
    const UnitVec3<Velocity>& v_gcrs,
    const UnitVec3<Length>& r_lvlh,
    UnitVec3<Length>& r_out
) noexcept {
    const scalar r_norm_val = norm(r_gcrs).get();
    if (r_norm_val <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const auto r_hat = normalize(r_gcrs);
    const auto h_vec = cross(r_gcrs, v_gcrs);
    const scalar h_norm_val = norm(h_vec).get();

    if (h_norm_val <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const auto h_hat = normalize(h_vec);
    const auto theta_hat = cross(h_hat, r_hat);

    const Vec3 result =
        r_hat.data * r_lvlh.x() +
        theta_hat.data * r_lvlh.y() +
        h_hat.data * r_lvlh.z();

    r_out = UnitVec3<Length>{result};

    return std::isfinite(r_out.x()) &&
           std::isfinite(r_out.y()) &&
           std::isfinite(r_out.z());
}

[[nodiscard]] constexpr bool gcrs_to_lvlh(
    const UnitVec3<Length>& r_gcrs,
    const UnitVec3<Velocity>& v_gcrs,
    const UnitVec3<Length>& r_in,
    UnitVec3<Length>& r_lvlh
) noexcept {
    const scalar r_norm_val = norm(r_gcrs).get();
    if (r_norm_val <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const auto r_hat = normalize(r_gcrs);
    const auto h_vec = cross(r_gcrs, v_gcrs);
    const scalar h_norm_val = norm(h_vec).get();

    if (h_norm_val <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const auto h_hat = normalize(h_vec);
    const auto theta_hat = cross(h_hat, r_hat);

    r_lvlh = UnitVec3<Length>{Vec3{
        dot(r_hat.data, r_in.data),
        dot(theta_hat.data, r_in.data),
        dot(h_hat.data, r_in.data)
    }};

    return std::isfinite(r_lvlh.x()) &&
           std::isfinite(r_lvlh.y()) &&
           std::isfinite(r_lvlh.z());
}

// ═══════════════════════════════════════════
// [6] BODY ↔ GCRS
// Quaternion rotation — extreme guard সহ
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool body_to_gcrs(
    const Quaternion& q,
    const UnitVec3<Length>& r_body,
    UnitVec3<Length>& r_gcrs
) noexcept {
    const scalar q_norm_sq = norm_sq(q);
    if (q_norm_sq < kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    if (norm_sq(r_body).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    Quaternion qn = q;
    qn = qn.normalize();
    const Vec3 r = r_body.data;

    const scalar qw = qn.get_w();
    const scalar qx = qn.get_x();
    const scalar qy = qn.get_y();
    const scalar qz = qn.get_z();

    const scalar r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
    const scalar r12 = 2.0f * (qx * qy - qw * qz);
    const scalar r13 = 2.0f * (qx * qz + qw * qy);

    const scalar r21 = 2.0f * (qx * qy + qw * qz);
    const scalar r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
    const scalar r23 = 2.0f * (qy * qz - qw * qx);

    const scalar r31 = 2.0f * (qx * qz - qw * qy);
    const scalar r32 = 2.0f * (qy * qz + qw * qx);
    const scalar r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    const Vec3 result{
        r11 * r.x() + r12 * r.y() + r13 * r.z(),
        r21 * r.x() + r22 * r.y() + r23 * r.z(),
        r31 * r.x() + r32 * r.y() + r33 * r.z()
    };

    r_gcrs = UnitVec3<Length>{result};

    return std::isfinite(r_gcrs.x()) &&
           std::isfinite(r_gcrs.y()) &&
           std::isfinite(r_gcrs.z());
}

[[nodiscard]] constexpr bool gcrs_to_body(
    const Quaternion& q,
    const UnitVec3<Length>& r_gcrs,
    UnitVec3<Length>& r_body
) noexcept {
    const scalar q_norm_sq = norm_sq(q);
    if (q_norm_sq < kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    if (norm_sq(r_gcrs).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Quaternion qn = q.normalize();
    const Vec3 r = r_gcrs.data;

    const scalar qw = qn.get_w();
    const scalar qx = qn.get_x();
    const scalar qy = qn.get_y();
    const scalar qz = qn.get_z();

    const scalar r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
    const scalar r12 = 2.0f * (qx * qy - qw * qz);
    const scalar r13 = 2.0f * (qx * qz + qw * qy);

    const scalar r21 = 2.0f * (qx * qy + qw * qz);
    const scalar r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
    const scalar r23 = 2.0f * (qy * qz - qw * qx);

    const scalar r31 = 2.0f * (qx * qz - qw * qy);
    const scalar r32 = 2.0f * (qy * qz + qw * qx);
    const scalar r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    // Transpose rotation
    const Vec3 result{
        r11 * r.x() + r21 * r.y() + r31 * r.z(),
        r12 * r.x() + r22 * r.y() + r32 * r.z(),
        r13 * r.x() + r23 * r.y() + r33 * r.z()
    };

    r_body = UnitVec3<Length>{result};

    return std::isfinite(r_body.x()) &&
           std::isfinite(r_body.y()) &&
           std::isfinite(r_body.z());
}

//---------------------------------------------
//---------------------------------------------

// ═══════════════════════════════════════════
// [7] GEODETIC ↔ CARTESIAN
// Latitude/Longitude/Altitude ↔ ECEF
// Extreme: pole, equator, negative altitude
// ═══════════════════════════════════════════

constexpr std::size_t kMaxBowringIter{10};

[[nodiscard]] constexpr bool geodetic_from_cartesian(
    const UnitVec3<Length>& r_cart,
    const scalar equatorial_radius,
    const scalar flattening,
    Angle& latitude,
    Angle& longitude,
    Length& altitude
) noexcept {
    const scalar x = r_cart.x();
    const scalar y = r_cart.y();
    const scalar z = r_cart.z();

    const scalar r_xy = std::sqrt(x * x + y * y);

    // Extreme: polar axis — no horizontal component
    if (r_xy <= kNormSqThreshold) APSA_UNLIKELY {
        latitude = Angle{(z >= scalar{0.0f}) ? kFrameHalfPi : -kFrameHalfPi};
        longitude = Angle{0.0f};
        altitude = Length{std::abs(z) - equatorial_radius * (1.0f - flattening)};
        return true;
    }

    const scalar lambda = std::atan2(y, x);
    const scalar e2 = flattening * (2.0f - flattening);
    const scalar ep2 = e2 / (1.0f - e2);

    scalar beta = std::atan2(z, (1.0f - flattening) * r_xy);
    scalar phi = beta;
    bool converged{false};

    for (std::size_t i{0}; i < kMaxBowringIter; ++i) {
        const scalar sin_beta = std::sin(beta);
        const scalar cos_beta = std::cos(beta);

        phi = std::atan2(
            z + ep2 * equatorial_radius * sin_beta * sin_beta * sin_beta,
            r_xy - e2 * equatorial_radius * cos_beta * cos_beta * cos_beta
        );

        const scalar new_beta = std::atan2(
            (1.0f - flattening) * std::sin(phi), std::cos(phi)
        );

        if (std::abs(new_beta - beta) < scalar{1e-12}) {
            beta = new_beta;
            converged = true;
            break;
        }

        beta = new_beta;
    }

    if (!converged) APSA_UNLIKELY {
        return false;
    }

    const scalar sin_phi = std::sin(phi);
    const scalar cos_phi = std::cos(phi);

    if (std::abs(cos_phi) <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar N = equatorial_radius /
        std::sqrt(1.0f - e2 * sin_phi * sin_phi);

    const scalar alt = r_xy / cos_phi - N;

    if (alt < -equatorial_radius) APSA_UNLIKELY {
        return false;
    }

    latitude = Angle{phi};
    longitude = Angle{lambda};
    altitude = Length{alt};

    return true;
}


[[nodiscard]] constexpr bool geodetic_from_itrs(
    const UnitVec3<Length>& r_itrs,
    const CentralBody& body,
    Angle& latitude,
    Angle& longitude,
    Length& altitude
) noexcept {
    return geodetic_from_cartesian(
        r_itrs,
        body.radius.get(),
        body.flattening.get(),
        latitude,
        longitude,
        altitude
    );
}


[[nodiscard]] constexpr bool geodetic_to_cartesian(
    const Angle latitude,
    const Angle longitude,
    const Length altitude,
    const scalar equatorial_radius,
    const scalar flattening,
    UnitVec3<Length>& r_cart
) noexcept {
    const scalar lat = latitude.get();
    const scalar lon = longitude.get();
    const scalar h = altitude.get();

    if (h < scalar{-equatorial_radius}) APSA_UNLIKELY {
        return false;
    }

    const scalar e2 = flattening * (2.0f - flattening);
    const scalar sin_lat = std::sin(lat);
    const scalar cos_lat = std::cos(lat);
    const scalar N = equatorial_radius /
        std::sqrt(1.0f - e2 * sin_lat * sin_lat);

    const scalar x = (N + h) * cos_lat * std::cos(lon);
    const scalar y = (N + h) * cos_lat * std::sin(lon);
    const scalar z = (N * (1.0f - e2) + h) * sin_lat;

    r_cart = UnitVec3<Length>{Vec3{x, y, z}};

    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// ═══════════════════════════════════════════
// [8] NED FRAME
// North-East-Down
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool itrs_to_ned(
    const UnitVec3<Length>& r_itrs,
    const UnitVec3<Length>& r_ref,
    const CentralBody& body,
    UnitVec3<Length>& r_ned
) noexcept {
    Angle lat{}, lon{};
    Length alt{};

    if (!geodetic_from_cartesian(r_ref, body.radius.get(),
                                 body.flattening.get(), lat, lon, alt)) {
        return false;
    }

    const scalar sin_phi = std::sin(lat.get());
    const scalar cos_phi = std::cos(lat.get());
    const scalar sin_lam = std::sin(lon.get());
    const scalar cos_lam = std::cos(lon.get());

    const scalar dx = r_itrs.x() - r_ref.x();
    const scalar dy = r_itrs.y() - r_ref.y();
    const scalar dz = r_itrs.z() - r_ref.z();

    r_ned = UnitVec3<Length>{Vec3{
        -sin_phi * cos_lam * dx - sin_phi * sin_lam * dy + cos_phi * dz,
        -sin_lam * dx + cos_lam * dy,
        -cos_phi * cos_lam * dx - cos_phi * sin_lam * dy - sin_phi * dz
    }};

    return std::isfinite(r_ned.x()) &&
           std::isfinite(r_ned.y()) &&
           std::isfinite(r_ned.z());
}

[[nodiscard]] constexpr bool ned_to_itrs(
    const UnitVec3<Length>& r_ned,
    const UnitVec3<Length>& r_ref,
    const CentralBody& body,
    UnitVec3<Length>& r_itrs
) noexcept {
    Angle lat{}, lon{};
    Length alt{};

    if (!geodetic_from_cartesian(r_ref, body.radius.get(),
                                 body.flattening.get(), lat, lon, alt)) {
        return false;
    }

    const scalar sin_phi = std::sin(lat.get());
    const scalar cos_phi = std::cos(lat.get());
    const scalar sin_lam = std::sin(lon.get());
    const scalar cos_lam = std::cos(lon.get());

    const scalar n = r_ned.x();
    const scalar e = r_ned.y();
    const scalar d = r_ned.z();

    r_itrs = UnitVec3<Length>{Vec3{
        r_ref.x() - sin_phi * cos_lam * n - sin_lam * e - cos_phi * cos_lam * d,
        r_ref.y() - sin_phi * sin_lam * n + cos_lam * e - cos_phi * sin_lam * d,
        r_ref.z() + cos_phi * n - sin_phi * d
    }};

    return std::isfinite(r_itrs.x()) &&
           std::isfinite(r_itrs.y()) &&
           std::isfinite(r_itrs.z());
}

// ═══════════════════════════════════════════
// [9] MARS FRAME
// ICRF ↔ IAU_MARS
// ═══════════════════════════════════════════

constexpr scalar kMarsPoleRARef{317.269202f};
constexpr scalar kMarsPoleRARate{-0.10927547f};
constexpr scalar kMarsPoleDecRef{54.432516f};
constexpr scalar kMarsPoleDecRate{-0.05827105f};
constexpr scalar kMarsPrimeMeridianRef{84.627100f};
constexpr scalar kMarsPrimeMeridianRate{350.8919830f};
constexpr scalar kDegreeToRadian{0.017453292519943295f};

[[nodiscard]] constexpr bool compute_mars_orientation(
    const Dimensionless D_u,
    const Dimensionless T,
    Angle& alpha_0,
    Angle& delta_0,
    Angle& W
) noexcept {
    const scalar d = D_u.get();
    const scalar t = T.get();

    if (!std::isfinite(d) || !std::isfinite(t)) APSA_UNLIKELY {
        return false;
    }

    const scalar alpha_val = (kMarsPoleRARef + kMarsPoleRARate * t);
    const scalar delta_val = (kMarsPoleDecRef + kMarsPoleDecRate * t);
    const scalar w_val = (kMarsPrimeMeridianRef + kMarsPrimeMeridianRate * d);

    // Wrap angles
    alpha_0 = Angle{std::fmod(alpha_val * kDegreeToRadian, kFrameTwoPi)};
    delta_0 = Angle{delta_val * kDegreeToRadian};
    W = Angle{std::fmod(w_val * kDegreeToRadian, kFrameTwoPi)};

    return std::isfinite(alpha_0.get()) &&
           std::isfinite(delta_0.get()) &&
           std::isfinite(W.get());
}

[[nodiscard]] constexpr bool icrf_to_iau_mars(
    const Angle alpha_0,
    const Angle delta_0,
    const Angle W,
    const UnitVec3<Length>& r_icrf,
    UnitVec3<Length>& r_iau
) noexcept {
    if (norm_sq(r_icrf).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Mat3 M =
        r3(Radian{W.get()}) *
        r1(Radian{kFrameHalfPi - delta_0.get()}) *
        r3(Radian{alpha_0.get() + kFrameHalfPi});

    r_iau = UnitVec3<Length>{M * r_icrf.data};

    return std::isfinite(r_iau.x()) &&
           std::isfinite(r_iau.y()) &&
           std::isfinite(r_iau.z());
}

[[nodiscard]] constexpr bool iau_mars_to_icrf(
    const Angle alpha_0,
    const Angle delta_0,
    const Angle W,
    const UnitVec3<Length>& r_iau,
    UnitVec3<Length>& r_icrf
) noexcept {
    if (norm_sq(r_iau).get() <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const Mat3 M =
        r3(Radian{W.get()}) *
        r1(Radian{kFrameHalfPi - delta_0.get()}) *
        r3(Radian{alpha_0.get() + kFrameHalfPi});

    r_icrf = UnitVec3<Length>{transpose(M) * r_iau.data};

    return std::isfinite(r_icrf.x()) &&
           std::isfinite(r_icrf.y()) &&
           std::isfinite(r_icrf.z());
}

[[nodiscard]] constexpr bool iau_mars_to_ned(
    const UnitVec3<Length>& r_iau,
    const UnitVec3<Length>& r_ref,
    UnitVec3<Length>& r_ned
) noexcept {
    Angle lat{}, lon{};
    Length alt{};

    if (!geodetic_from_cartesian(r_ref, kMarsRadius.get(),
                                 kMarsFlattening.get(), lat, lon, alt)) {
        return false;
    }

    const scalar sin_phi = std::sin(lat.get());
    const scalar cos_phi = std::cos(lat.get());
    const scalar sin_lam = std::sin(lon.get());
    const scalar cos_lam = std::cos(lon.get());

    const scalar dx = r_iau.x() - r_ref.x();
    const scalar dy = r_iau.y() - r_ref.y();
    const scalar dz = r_iau.z() - r_ref.z();

    r_ned = UnitVec3<Length>{Vec3{
        -sin_phi * cos_lam * dx - sin_phi * sin_lam * dy + cos_phi * dz,
        -sin_lam * dx + cos_lam * dy,
        -cos_phi * cos_lam * dx - cos_phi * sin_lam * dy - sin_phi * dz
    }};

    return std::isfinite(r_ned.x()) &&
           std::isfinite(r_ned.y()) &&
           std::isfinite(r_ned.z());
}

[[nodiscard]] constexpr bool ned_to_iau_mars(
    const UnitVec3<Length>& r_ned,
    const UnitVec3<Length>& r_ref,
    UnitVec3<Length>& r_iau
) noexcept {
    Angle lat{}, lon{};
    Length alt{};

    if (!geodetic_from_cartesian(r_ref, kMarsRadius.get(),
                                 kMarsFlattening.get(), lat, lon, alt)) {
        return false;
    }

    const scalar sin_phi = std::sin(lat.get());
    const scalar cos_phi = std::cos(lat.get());
    const scalar sin_lam = std::sin(lon.get());
    const scalar cos_lam = std::cos(lon.get());

    const scalar n = r_ned.x();
    const scalar e = r_ned.y();
    const scalar d = r_ned.z();

    r_iau = UnitVec3<Length>{Vec3{
        r_ref.x() - sin_phi * cos_lam * n - sin_lam * e - cos_phi * cos_lam * d,
        r_ref.y() - sin_phi * sin_lam * n + cos_lam * e - cos_phi * sin_lam * d,
        r_ref.z() + cos_phi * n - sin_phi * d
    }};

    return std::isfinite(r_iau.x()) &&
           std::isfinite(r_iau.y()) &&
           std::isfinite(r_iau.z());
}

// ═══════════════════════════════════════════
// [10] LIGHT TIME
// Signal travel time — extreme guard
// ═══════════════════════════════════════════

// constexpr scalar kLightSpeed{299792458.0f};
constexpr scalar kLightTimeTolerance{1e-12};
constexpr std::size_t kMaxLightTimeIter{10};

[[nodiscard]] constexpr bool light_time_correction(
    const UnitVec3<Length>& r_sc,
    const UnitVec3<Length>& r_station,
    Time& tau
) noexcept {
    const scalar dx = r_sc.x() - r_station.x();
    const scalar dy = r_sc.y() - r_station.y();
    const scalar dz = r_sc.z() - r_station.z();

    const scalar dist_sq = dx*dx + dy*dy + dz*dz;

    if (dist_sq <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar dist = std::sqrt(dist_sq);

    if (!std::isfinite(dist)) APSA_UNLIKELY {
        return false;
    }

    const scalar tau_val = dist / kLightSpeed;

    if (!std::isfinite(tau_val) || tau_val <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    tau = Time{tau_val};
    return true;
}

//-----------------------------------------------------------
//-----------------------------------------------------------

// ═══════════════════════════════════════════
// [11] EOP BINARY RECORD
// Earth orientation parameter — record type
// ═══════════════════════════════════════════

struct BinaryHeader {
    std::uint32_t magic{0U};
    std::uint16_t version{0U};
    std::uint32_t record_size{0U};
    std::uint32_t record_count{0U};
    std::uint32_t crc32{0U};
};

struct EOPRecord {
    std::uint64_t epoch{0U};
    scalar xp{0.0f};
    scalar yp{0.0f};
    scalar dUT1{0.0f};
    scalar dPsi{0.0f};
    scalar dEps{0.0f};
    std::uint64_t delta_AT{0U};
};

inline constexpr std::uint32_t kBinaryMagic{0x454F5031U};
inline constexpr std::uint16_t kBinaryVersion{1U};

// ═══════════════════════════════════════════
// [12] EOP SELECT
// Binary search — closest previous epoch
// Extreme: empty, out-of-range clamp
// ═══════════════════════════════════════════

[[nodiscard]] bool eop_select(
    const std::array<EOPRecord, 256>& records,
    const std::size_t record_count,
    const std::uint64_t epoch,
    EOPData& eop
) noexcept {
    if (record_count == 0U) APSA_UNLIKELY {
        return false;
    }

    if (record_count > records.size()) APSA_UNLIKELY {
        return false;
    }

    // Before first epoch — use first
    if (epoch < records[0].epoch) {
        const EOPRecord& rec = records[0];
        eop.xp = Angle{rec.xp};
        eop.yp = Angle{rec.yp};
        eop.dUT1 = Time{rec.dUT1};
        eop.dPsi = Angle{rec.dPsi};
        eop.dEps = Angle{rec.dEps};
        eop.delta_AT = Time{static_cast<scalar>(rec.delta_AT)};
        return true;
    }

    // After last epoch — clamp to latest
    if (epoch >= records[record_count - 1U].epoch) {
        const EOPRecord& rec = records[record_count - 1U];
        eop.xp = Angle{rec.xp};
        eop.yp = Angle{rec.yp};
        eop.dUT1 = Time{rec.dUT1};
        eop.dPsi = Angle{rec.dPsi};
        eop.dEps = Angle{rec.dEps};
        eop.delta_AT = Time{static_cast<scalar>(rec.delta_AT)};
        return true;
    }

    // Binary search
    std::size_t low{0U};
    std::size_t high{record_count - 1U};

    while (high - low > 1U) {
        const std::size_t mid = low + (high - low) / 2U;
        if (records[mid].epoch <= epoch) {
            low = mid;
        } else {
            high = mid;
        }
    }

    const EOPRecord& selected = records[low];
    eop.xp = Angle{selected.xp};
    eop.yp = Angle{selected.yp};
    eop.dUT1 = Time{selected.dUT1};
    eop.dPsi = Angle{selected.dPsi};
    eop.dEps = Angle{selected.dEps};
    eop.delta_AT = Time{static_cast<scalar>(selected.delta_AT)};

    return true;
}

// ═══════════════════════════════════════════
// [13] NUTATION TYPES
// Precession-nutation series structures
// ═══════════════════════════════════════════

constexpr std::size_t kLuniSolarTerms{1320};
constexpr std::size_t kPlanetaryTerms{687};

struct NutationTerm {
    scalar a_s0{0.0f}, a_c0{0.0f};
    scalar a_s1{0.0f}, a_c1{0.0f};
    scalar a_s2{0.0f}, a_c2{0.0f};
    scalar m1{0.0f}, m2{0.0f}, m3{0.0f}, m4{0.0f}, m5{0.0f};
};

struct PlanetaryTerm {
    scalar a_s0{0.0f}, a_c0{0.0f};
    scalar m1{0.0f}, m2{0.0f}, m3{0.0f}, m4{0.0f}, m5{0.0f};
    scalar m6{0.0f}, m7{0.0f}, m8{0.0f}, m9{0.0f};
};

struct STerm {
    scalar c{0.0f}, d{0.0f};
    scalar m1{0.0f}, m2{0.0f}, m3{0.0f}, m4{0.0f}, m5{0.0f};
};

struct NutationTable {
    std::array<NutationTerm, kLuniSolarTerms> luni_solar{};
    std::array<PlanetaryTerm, kPlanetaryTerms> planetary{};
    std::array<STerm, 66> s_terms{};
};

// ═══════════════════════════════════════════
// [14] FRAME SYSTEM
// Full pipeline container
// ═══════════════════════════════════════════

struct FrameSystem {
    CentralBody body{};
    EOPData eop{};
    NutationTable nutation{};
    TimeState time{};
};

enum class FrameError : std::uint8_t {
    OK,
    INVALID_INPUT,
    TIME_INVALID,
    FRAME_SINGULAR,
    CONVERGENCE_FAIL,
    DATA_OUT_OF_RANGE
};

[[nodiscard]] bool frame_system_load(
    const CentralBody& body,
    const EOPData& eop,
    const NutationTable& nutation,
    FrameSystem& system
) noexcept {
    system.body = body;
    system.eop = eop;
    system.nutation = nutation;

    return true;
}

// ═══════════════════════════════════════════
// [15] TESTS
// Frame/time sanity — extreme guard
// ═══════════════════════════════════════════

[[nodiscard]] bool test_time_chain() noexcept {
    Time utc{0.0f};
    Time tai{}, tt{}, tdb{};

    if (!utc_tai(utc, Time{37.0f}, tai)) return false;
    if (!tai_tt(tai, tt)) return false;
    if (!tt_tdb(tt, kTDBTerms, tdb)) return false;

    if (tdb.get() < tt.get()) return false;
    return true;
}

[[nodiscard]] bool test_era_known_value() noexcept {
    Time ut1{0.0f};
    Radian era{};

    if (!earth_rotation_angle(ut1, era)) return false;

    const scalar expected = 4.894961212823756f;
    if (std::abs(era.get() - expected) > 1e-9) return false;

    return true;
}

[[nodiscard]] bool test_body_roundtrip() noexcept {
    const Quaternion q{0.7071067811865476f, 0.0f, 0.0f, 0.7071067811865476f};
    const UnitVec3<Length> r_body{Vec3{1.0f, 2.0f, 3.0f}};

    UnitVec3<Length> r_gcrs{};
    if (!body_to_gcrs(q, r_body, r_gcrs)) return false;

    UnitVec3<Length> r_back{};
    if (!gcrs_to_body(q, r_gcrs, r_back)) return false;

    if (std::abs(r_back.x() - r_body.x()) > 1e-9) return false;
    if (std::abs(r_back.y() - r_body.y()) > 1e-9) return false;
    if (std::abs(r_back.z() - r_body.z()) > 1e-9) return false;

    return true;
}

[[nodiscard]] bool test_mars_roundtrip() noexcept {
    const Angle alpha{0.1f};
    const Angle delta{0.2f};
    const Angle W{0.3f};

    const UnitVec3<Length> r_icrf{Vec3{3396190.0f, 1000.0f, 2000.0f}};

    UnitVec3<Length> r_iau{};
    if (!icrf_to_iau_mars(alpha, delta, W, r_icrf, r_iau)) return false;

    UnitVec3<Length> r_back{};
    if (!iau_mars_to_icrf(alpha, delta, W, r_iau, r_back)) return false;

    if (std::abs(r_back.x() - r_icrf.x()) > 1e-6) return false;
    if (std::abs(r_back.y() - r_icrf.y()) > 1e-6) return false;
    if (std::abs(r_back.z() - r_icrf.z()) > 1e-6) return false;

    return true;
}

[[nodiscard]] bool test_light_time_converges() noexcept {
    const UnitVec3<Length> r_sc{Vec3{1.5e11, 0.0f, 0.0f}};
    const UnitVec3<Length> r_station{Vec3{0.0f, 0.0f, 0.0f}};

    Time tau{};
    if (!light_time_correction(r_sc, r_station, tau)) return false;

    if (tau.get() <= 0.0f) return false;

    const scalar expected = 1.5e11 / 299792458.0f;
    if (std::abs(tau.get() - expected) > 1e-6) return false;

    return true;
}

[[nodiscard]] bool test_boundary_cases() noexcept {
    Time tai{};
    if (utc_tai(Time{-1.0f}, Time{37.0f}, tai)) return false;

    Radian era{};
    if (earth_rotation_angle(Time{-1.0f}, era)) return false;

    return true;
}

[[nodiscard]] bool run_frame_system_tests() noexcept {
    if (!test_time_chain()) return false;
    if (!test_era_known_value()) return false;
    if (!test_body_roundtrip()) return false;
    if (!test_mars_roundtrip()) return false;
    if (!test_light_time_converges()) return false;
    if (!test_boundary_cases()) return false;
    return true;
}

//---------------------------------------------------------
//---------------------------------------------------------