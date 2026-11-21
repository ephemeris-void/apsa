#pragma once

#include "apsa_physics_config.hpp"
#include "core/apsa_physics_math.hpp"
#include "core/frame_engine.hpp"
#include "core/apsa_physics_gravity.hpp"
#include "core/frame_engine.hpp"
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"

#include <array>
#include <cmath>
#include <algorithm>

[[nodiscard]] inline bool sixdof_translation_body(
    const Mass& mass,
    const UnitVec3<Force>& F_total,
    const AngularVel& omega_x, const AngularVel& omega_y, const AngularVel& omega_z,
    const UnitVec3<Velocity>& v,
    UnitVec3<Velocity>& v_dot_out
) noexcept {
    (void)omega_x; (void)omega_y; (void)omega_z; (void)v;
    const scalar m = mass.get();
    if (m <= kNormSqThreshold) return false;
    v_dot_out = UnitVec3<Velocity>{F_total.data * (1.0f / m)};
    return true;
}

[[nodiscard]] inline bool sixdof_translation_body(
    const Mass& mass,
    const UnitVec3<Force>& F_total,
    const UnitVec3<Velocity>& v_body,
    UnitVec3<Velocity>& v_dot_out
) noexcept {
    (void)v_body;
    const scalar m = mass.get();
    if (m <= kNormSqThreshold) return false;
    v_dot_out = UnitVec3<Velocity>{F_total.data * (1.0f / m)};
    return true;
}

// ═══════════════════════════════════════════
// APSA PHYSICS — RIGID BODY
// 6DOF Dynamics | Force | Torque | Mass | Inertia
// Vehicle-free | Planet-free | Parameter-driven
// DO-178C Level A | No Hardcoded Value
// ═══════════════════════════════════════════

// ═══════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════

constexpr scalar kMaxOmegaSq{1.0e8};
constexpr std::size_t kMaxForceContributions{16U};
constexpr std::size_t kMaxTorqueContributions{32U};
constexpr std::size_t kMaxMassElements{16U};
constexpr std::size_t kMaxSloshMasses{4U};
constexpr std::size_t kMaxRcsThrusters{12U};
constexpr std::size_t kMaxFlexibleModes{8U};
constexpr std::size_t kMaxSloshModes{4U};

constexpr scalar kEulerSingularityThreshold{0.1f};
constexpr scalar k6dofMinMass{1e-9};
constexpr scalar kTorqueMinRadius{1e3};
constexpr scalar kPAxisMinMass{1e-9};
constexpr scalar kPAxisMinInertia{1e-15};
constexpr scalar kCgMinMass{1e-6};
constexpr scalar kCgMinTotalMass{1e-3};
constexpr scalar kInertiaMinMass{1e-6};
constexpr scalar kInertiaMinValue{1e-12};
constexpr scalar kInertiaMinDet{1e-20};

// ═══════════════════════════════════════════
// SKEW SYMMETRIC
// ═══════════════════════════════════════════

[[nodiscard]] constexpr Mat3 skew_symmetric(
    const scalar vx,
    const scalar vy,
    const scalar vz
) noexcept {
    return Mat3{{
        0.0f,  -vz,  vy,
        vz,    0.0f, -vx,
       -vy,   vx,   0.0f
    }};
}

// ═══════════════════════════════════════════
// NEWTON-EULER TRANSLATION — Body Frame
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool newton_euler_translation(
    const Mass mass,
    const UnitVec3<Velocity>& velocity_body,
    const UnitVec3<AngularVel>& angular_velocity,
    const UnitVec3<Force>& force_body,
    UnitVec3<Acceleration>& acceleration
) noexcept {
    const scalar m = mass.get();

    if (m <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar v_mag = norm(velocity_body).get();
    const scalar w_mag = norm(angular_velocity).get();

    if (v_mag < kNormSqThreshold && w_mag < kNormSqThreshold) {
        acceleration = UnitVec3<Acceleration>{
            force_body.data / m
        };
        return true;
    }

    const auto coriolis = cross(angular_velocity, velocity_body);
    const Vec3 net_force = (force_body.data - (coriolis.data * m));
    const Vec3 accel = net_force / m;

    acceleration = UnitVec3<Acceleration>{accel};
    return true;
}

// ═══════════════════════════════════════════
// NEWTON-EULER ROTATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool newton_euler_rotation(
    const Mat3& inertia,
    const Mat3& inertia_dot,
    const UnitVec3<AngularVel>& omega,
    const UnitVec3<Torque>& moment,
    UnitVec3<AngularAccel>& angular_accel
) noexcept {
    const scalar det = determinant(inertia);

    if (std::abs(det) < kDetThreshold) APSA_UNLIKELY {
        return false;
    }

    Mat3 inv_inertia{};
    const bool inv_ok = inverse(inertia, inv_inertia);
    if (!inv_ok) APSA_UNLIKELY {
        return false;
    }

    const Vec3 w = omega.data;

    const scalar w_sq = dot(w, w);
    if (w_sq > kMaxOmegaSq) APSA_UNLIKELY {
        return false;
    }

    if (!std::isfinite(w.x()) ||
        !std::isfinite(w.y()) ||
        !std::isfinite(w.z())) {
        APSA_UNLIKELY { return false; }
    }

    const Vec3 I_w = inertia * w;
    if (!std::isfinite(I_w.x()) ||
        !std::isfinite(I_w.y()) ||
        !std::isfinite(I_w.z())) {
        APSA_UNLIKELY { return false; }
    }

    const Vec3 gyro = cross(w, I_w);
    if (!std::isfinite(gyro.x()) ||
        !std::isfinite(gyro.y()) ||
        !std::isfinite(gyro.z())) {
        APSA_UNLIKELY { return false; }
    }

    const Vec3 I_dot_w = inertia_dot * w;
    if (!std::isfinite(I_dot_w.x()) ||
        !std::isfinite(I_dot_w.y()) ||
        !std::isfinite(I_dot_w.z())) {
        APSA_UNLIKELY { return false; }
    }

    const Vec3 net = moment.data - gyro - I_dot_w;
    const Vec3 alpha = inv_inertia * net;

    if (!std::isfinite(alpha.x()) ||
        !std::isfinite(alpha.y()) ||
        !std::isfinite(alpha.z())) {
        APSA_UNLIKELY { return false; }
    }

    angular_accel = UnitVec3<AngularAccel>{alpha};
    return true;
}

// ═══════════════════════════════════════════
// DCM DERIVATIVE + ORTHOGONALITY
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool dcm_derivative(
    const Mat3& rotation,
    const UnitVec3<AngularVel>& omega,
    Mat3& rotation_dot
) noexcept {
    const scalar wx = omega.x();
    const scalar wy = omega.y();
    const scalar wz = omega.z();

    const Mat3 omega_cross = Mat3{{
        0.0f, -wz,  wy,
        wz,   0.0f, -wx,
       -wy,   wx,   0.0f
    }};

    rotation_dot = rotation * omega_cross;

    return true;
}

[[nodiscard]] constexpr bool dcm_orthogonality_correct(
    const Mat3& rotation,
    Mat3& corrected
) noexcept {
    const Mat3 R_T = transpose(rotation);
    const Mat3 R_T_R = R_T * rotation;

    Mat3 inv_sqrt{};
    const bool inv_ok = inverse(R_T_R, inv_sqrt);
    if (!inv_ok) APSA_UNLIKELY {
        return false;
    }

    corrected = rotation * inv_sqrt;

    return true;
}

// ═══════════════════════════════════════════
// SIXDOF DERIVATIVE
// ═══════════════════════════════════════════

struct SixDofDerivative {
    UnitVec3<Velocity> v_dot{};
    AngularVel alpha_x{0.0f};
    AngularVel alpha_y{0.0f};
    AngularVel alpha_z{0.0f};
};

// forward declare InertiaTensorPhysics (defined later)
struct InertiaTensorPhysics;

[[nodiscard]] bool sixdof_derivative(
    const Mass mass,
    const MassFlowRate m_dot,
    const InertiaTensorPhysics& I,
    const InertiaTensorPhysics& I_dot,
    const UnitVec3<Force>& F_total,
    const UnitVec3<Torque>& M_total,
    const UnitVec3<Velocity>& v,
    const AngularVel omega_x,
    const AngularVel omega_y,
    const AngularVel omega_z,
    SixDofDerivative& deriv
) noexcept;

// ═══════════════════════════════════════════
// FORCE MODELS — Universal
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool force_thrust(
    const Force thrust_magnitude,
    const Vec3& thrust_direction_body,
    const Quaternion& attitude,
    UnitVec3<Force>& F_thrust
) noexcept {
    if (thrust_magnitude.get() < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const Vec3 direction_inertial = rotate(attitude, thrust_direction_body);

    const scalar magnitude = thrust_magnitude.get();

    F_thrust = UnitVec3<Force>{Vec3{
        magnitude * direction_inertial.x(),
        magnitude * direction_inertial.y(),
        magnitude * direction_inertial.z()
    }};

    if (!std::isfinite(F_thrust.x()) ||
        !std::isfinite(F_thrust.y()) ||
        !std::isfinite(F_thrust.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool force_aero_body(
    const scalar dynamic_pressure_val,
    const Area reference_area,
    const scalar C_A,
    const scalar C_Y,
    const scalar C_N,
    UnitVec3<Force>& F_aero_body
) noexcept {
    if (dynamic_pressure_val < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    if (reference_area.get() <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar qS = dynamic_pressure_val * reference_area.get();

    F_aero_body = UnitVec3<Force>{Vec3{
        -qS * C_A,
         qS * C_Y,
        -qS * C_N
    }};

    if (!std::isfinite(F_aero_body.x()) ||
        !std::isfinite(F_aero_body.y()) ||
        !std::isfinite(F_aero_body.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool force_aero_to_inertial(
    const UnitVec3<Force>& F_aero_body,
    const Quaternion& attitude,
    UnitVec3<Force>& F_aero_inertial
) noexcept {
    const Vec3 rotated = rotate(attitude, F_aero_body.data);

    F_aero_inertial = UnitVec3<Force>{rotated};

    if (!std::isfinite(rotated.x()) ||
        !std::isfinite(rotated.y()) ||
        !std::isfinite(rotated.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// FORCE GRAVITY J2
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool force_gravity_J2(
    const CentralBody& body,
    const Mass mass,
    const UnitVec3<Length>& r,
    UnitVec3<Force>& F_grav
) noexcept {
    const scalar r_sq = norm_sq(r).get();

    if (r_sq < kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar r_mag = std::sqrt(r_sq);
    const scalar r_cu = r_sq * r_mag;
    const scalar r_5 = r_cu * r_sq;

    if (r_cu < kNormSqThreshold || r_5 < kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }

    const scalar m = mass.get();
    const scalar mu = body.mu.get();
    const scalar j2 = body.j2.get();
    const scalar re = body.radius.get();

    const scalar two_body_factor = -mu * m / r_cu;

    const scalar z_sq = r.z() * r.z();
    const scalar z_sq_over_r_sq = z_sq / r_sq;

    const scalar j2_factor =
        scalar{1.5f} * mu * j2 * re * re * m / r_5;

    const scalar term_xy = scalar{5.0f} * z_sq_over_r_sq - scalar{1.0f};
    const scalar term_z = scalar{5.0f} * z_sq_over_r_sq - scalar{3.0f};

    const scalar Fx = two_body_factor * r.x() + j2_factor * r.x() * term_xy;
    const scalar Fy = two_body_factor * r.y() + j2_factor * r.y() * term_xy;
    const scalar Fz = two_body_factor * r.z() + j2_factor * r.z() * term_z;

    F_grav = UnitVec3<Force>{Vec3{Fx, Fy, Fz}};

    if (!std::isfinite(Fx) || !std::isfinite(Fy) || !std::isfinite(Fz)) {
        APSA_UNLIKELY { return false; }
    }

    return true;
}

// ═══════════════════════════════════════════
// FORCE RCS SUM
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool force_rcs_sum(
    const std::array<Vec3, kMaxRcsThrusters>& thruster_positions,
    const std::array<Vec3, kMaxRcsThrusters>& thruster_directions,
    const std::array<Force, kMaxRcsThrusters>& thruster_forces,
    const std::size_t thruster_count,
    UnitVec3<Force>& F_rcs
) noexcept {
    if (thruster_count == 0U || thruster_count > kMaxRcsThrusters) {
        APSA_UNLIKELY { return false; }
    }
    (void)thruster_positions;

    scalar Fx{0.0f};
    scalar Fy{0.0f};
    scalar Fz{0.0f};

    for (std::size_t i{0U}; i < thruster_count; ++i) {
        const scalar f_i = thruster_forces[i].get();
        const Vec3 dir = thruster_directions[i];

        Fx += f_i * dir.x();
        Fy += f_i * dir.y();
        Fz += f_i * dir.z();
    }

    F_rcs = UnitVec3<Force>{Vec3{Fx, Fy, Fz}};

    if (!std::isfinite(Fx) || !std::isfinite(Fy) || !std::isfinite(Fz)) {
        APSA_UNLIKELY { return false; }
    }

    return true;
}

// ═══════════════════════════════════════════
// ROTATING FRAME — CORIOLIS + CENTRIFUGAL
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool coriolis_acceleration(
    const UnitVec3<Velocity>& relative_velocity,
    const UnitVec3<AngularVel>& angular_velocity,
    UnitVec3<Acceleration>& coriolis
) noexcept {
    const auto cross_term = cross(angular_velocity, relative_velocity);

    const Vec3 a_cor = cross_term.data * scalar{-2.0f};

    if (!std::isfinite(a_cor.x()) ||
        !std::isfinite(a_cor.y()) ||
        !std::isfinite(a_cor.z())) {
        APSA_UNLIKELY { return false; }
    }

    coriolis = UnitVec3<Acceleration>{a_cor};
    return true;
}

[[nodiscard]] constexpr bool centrifugal_acceleration(
    const UnitVec3<Length>& position_vector,
    const UnitVec3<AngularVel>& angular_velocity,
    UnitVec3<Acceleration>& centrifugal
) noexcept {
    const auto first_cross = cross(angular_velocity, position_vector);
    const auto a_cf = cross(angular_velocity, first_cross);

    const Vec3 a_cf_final = a_cf.data * scalar{-1.0f};

    if (!std::isfinite(a_cf_final.x()) ||
        !std::isfinite(a_cf_final.y()) ||
        !std::isfinite(a_cf_final.z())) {
        APSA_UNLIKELY { return false; }
    }

    centrifugal = UnitVec3<Acceleration>{a_cf_final};
    return true;
}

[[nodiscard]] constexpr bool force_coriolis(
    const Mass mass,
    const UnitVec3<Velocity>& v_rel,
    const UnitVec3<AngularVel>& angular_velocity,
    UnitVec3<Force>& F_coriolis
) noexcept {
    const scalar m = mass.get();
    const scalar omega = angular_velocity.x();

    const Vec3 omega_vec{0.0f, 0.0f, omega};

    const auto cross_prod = cross(omega_vec, v_rel.data);

    const scalar factor = scalar{-2.0f} * m;

    F_coriolis = UnitVec3<Force>{Vec3{
        factor * cross_prod.x(),
        factor * cross_prod.y(),
        factor * cross_prod.z()
    }};

    if (!std::isfinite(F_coriolis.x()) ||
        !std::isfinite(F_coriolis.y()) ||
        !std::isfinite(F_coriolis.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool force_centrifugal(
    const Mass mass,
    const UnitVec3<Length>& r,
    const UnitVec3<AngularVel>& angular_velocity,
    UnitVec3<Force>& F_centrifugal
) noexcept {
    const scalar m = mass.get();
    const scalar omega = angular_velocity.x();

    const Vec3 omega_vec{0.0f, 0.0f, omega};
    const auto cross1 = cross(omega_vec, r.data);
    const auto cross2 = cross(omega_vec, cross1);

    const scalar factor = -m;

    F_centrifugal = UnitVec3<Force>{Vec3{
        factor * cross2.x(),
        factor * cross2.y(),
        factor * cross2.z()
    }};

    if (!std::isfinite(F_centrifugal.x()) ||
        !std::isfinite(F_centrifugal.y()) ||
        !std::isfinite(F_centrifugal.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool force_total(
    const UnitVec3<Force>& F_thrust,
    const UnitVec3<Force>& F_aero,
    const UnitVec3<Force>& F_grav,
    const UnitVec3<Force>& F_rcs,
    const UnitVec3<Force>& F_coriolis,
    const UnitVec3<Force>& F_centrifugal,
    UnitVec3<Force>& F_total
) noexcept {
    F_total = UnitVec3<Force>{Vec3{
        F_thrust.x() + F_aero.x() + F_grav.x() + F_rcs.x() +
            F_coriolis.x() + F_centrifugal.x(),
        F_thrust.y() + F_aero.y() + F_grav.y() + F_rcs.y() +
            F_coriolis.y() + F_centrifugal.y(),
        F_thrust.z() + F_aero.z() + F_grav.z() + F_rcs.z() +
            F_coriolis.z() + F_centrifugal.z()
    }};

    if (!std::isfinite(F_total.x()) ||
        !std::isfinite(F_total.y()) ||
        !std::isfinite(F_total.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// TORQUE MODELS
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool torque_thrust(
    const UnitVec3<Length>& r_engine_to_cg,
    const UnitVec3<Force>& F_thrust,
    UnitVec3<Torque>& M_thrust
) noexcept {
    const auto M = cross(r_engine_to_cg, F_thrust);

    M_thrust = UnitVec3<Torque>{M.data};

    if (!std::isfinite(M.x()) ||
        !std::isfinite(M.y()) ||
        !std::isfinite(M.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool torque_aero(
    const scalar dynamic_pressure_val,
    const Area reference_area,
    const Length reference_length,
    const scalar C_l,
    const scalar C_m,
    const scalar C_n,
    const UnitVec3<Length>& r_cp_to_cg,
    const UnitVec3<Force>& F_aero,
    UnitVec3<Torque>& M_aero
) noexcept {
    if (dynamic_pressure_val < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar qS = dynamic_pressure_val * reference_area.get();
    const scalar qSL = qS * reference_length.get();

    const scalar Mx_coeff = qSL * C_l;
    const scalar My_coeff = qSL * C_m;
    const scalar Mz_coeff = qSL * C_n;

    const auto M_offset = cross(r_cp_to_cg, F_aero);

    M_aero = UnitVec3<Torque>{Vec3{
        Mx_coeff + M_offset.x(),
        My_coeff + M_offset.y(),
        Mz_coeff + M_offset.z()
    }};

    if (!std::isfinite(M_aero.x()) ||
        !std::isfinite(M_aero.y()) ||
        !std::isfinite(M_aero.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool torque_rcs(
    const std::array<Vec3, kMaxRcsThrusters>& r_thruster_to_cg,
    const std::array<Vec3, kMaxRcsThrusters>& thruster_directions,
    const std::array<Force, kMaxRcsThrusters>& thruster_forces,
    const std::array<scalar, kMaxRcsThrusters>& thruster_commands,
    const std::size_t thruster_count,
    UnitVec3<Torque>& M_rcs
) noexcept {
    if (thruster_count == 0U || thruster_count > kMaxRcsThrusters) {
        APSA_UNLIKELY { return false; }
    }

    scalar Mx{0.0f};
    scalar My{0.0f};
    scalar Mz{0.0f};

    for (std::size_t i{0U}; i < thruster_count; ++i) {
        const scalar u_i = thruster_commands[i];
        const scalar F_i = thruster_forces[i].get() * u_i;

        const Vec3 r_i = r_thruster_to_cg[i];
        const Vec3 dir_i = thruster_directions[i];

        const Vec3 F_vec{
            F_i * dir_i.x(),
            F_i * dir_i.y(),
            F_i * dir_i.z()
        };

        const scalar tx = r_i.y() * F_vec.z() - r_i.z() * F_vec.y();
        const scalar ty = r_i.z() * F_vec.x() - r_i.x() * F_vec.z();
        const scalar tz = r_i.x() * F_vec.y() - r_i.y() * F_vec.x();

        Mx += tx;
        My += ty;
        Mz += tz;
    }

    M_rcs = UnitVec3<Torque>{Vec3{Mx, My, Mz}};

    if (!std::isfinite(Mx) || !std::isfinite(My) || !std::isfinite(Mz)) {
        APSA_UNLIKELY { return false; }
    }

    return true;
}

[[nodiscard]] constexpr bool torque_gravity_gradient(
    const UnitVec3<Length>& r_body_hat,
    const InertiaPrincipal& inertia,
    const CentralBody& body,
    UnitVec3<Torque>& M_gg
) noexcept {
    const scalar r_mag = norm(r_body_hat).get();

    if (r_mag < kTorqueMinRadius) APSA_UNLIKELY {
        return false;
    }

    const scalar r_cu = r_mag * r_mag * r_mag;

    if (r_cu < kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar coeff = scalar{3.0f} * body.mu.get() / r_cu;

    const scalar rx = r_body_hat.x();
    const scalar ry = r_body_hat.y();
    const scalar rz = r_body_hat.z();

    const scalar Ir_x = inertia.Ixx * rx;
    const scalar Ir_y = inertia.Iyy * ry;
    const scalar Ir_z = inertia.Izz * rz;

    const scalar Mx = coeff * (ry * Ir_z - rz * Ir_y);
    const scalar My = coeff * (rz * Ir_x - rx * Ir_z);
    const scalar Mz = coeff * (rx * Ir_y - ry * Ir_x);

    M_gg = UnitVec3<Torque>{Vec3{Mx, My, Mz}};

    if (!std::isfinite(Mx) || !std::isfinite(My) || !std::isfinite(Mz)) {
        APSA_UNLIKELY { return false; }
    }

    return true;
}

[[nodiscard]] constexpr bool torque_gimbal(
    const UnitVec3<Length>& r_gimbal,
    const Force engine_force,
    const Vec3& thrust_direction,
    const Force engine_force_dot,
    const Vec3& thrust_direction_dot,
    UnitVec3<Torque>& M_gimbal
) noexcept {
    const scalar F = engine_force.get();
    const scalar F_dot = engine_force_dot.get();

    const Vec3 T_vec{
        F * thrust_direction.x() + F_dot * thrust_direction_dot.x(),
        F * thrust_direction.y() + F_dot * thrust_direction_dot.y(),
        F * thrust_direction.z() + F_dot * thrust_direction_dot.z()
    };

    const auto M = cross(r_gimbal, UnitVec3<Force>{T_vec});

    M_gimbal = UnitVec3<Torque>{M.data};

    if (!std::isfinite(M.x()) ||
        !std::isfinite(M.y()) ||
        !std::isfinite(M.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool torque_total(
    const UnitVec3<Torque>& M_thrust,
    const UnitVec3<Torque>& M_aero,
    const UnitVec3<Torque>& M_rcs,
    const UnitVec3<Torque>& M_gg,
    const UnitVec3<Torque>& M_misalign,
    const UnitVec3<Torque>& M_gimbal,
    UnitVec3<Torque>& M_total
) noexcept {
    M_total = UnitVec3<Torque>{Vec3{
        M_thrust.x() + M_aero.x() + M_rcs.x() +
            M_gg.x() + M_misalign.x() + M_gimbal.x(),
        M_thrust.y() + M_aero.y() + M_rcs.y() +
            M_gg.y() + M_misalign.y() + M_gimbal.y(),
        M_thrust.z() + M_aero.z() + M_rcs.z() +
            M_gg.z() + M_misalign.z() + M_gimbal.z()
    }};

    if (!std::isfinite(M_total.x()) ||
        !std::isfinite(M_total.y()) ||
        !std::isfinite(M_total.z())) APSA_UNLIKELY {
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════
// MASS STATE
// ═══════════════════════════════════════════

struct MassState {
    Mass dry_mass{};
    Mass fuel_mass{};
    Mass oxidizer_mass{};
    Mass rcs_fuel_mass{};
    Mass pressurant_mass{};
};

[[nodiscard]] constexpr bool total_mass(
    const MassState& state,
    Mass& result
) noexcept {
    const scalar m_dry = state.dry_mass.get();
    const scalar m_f = state.fuel_mass.get();
    const scalar m_o = state.oxidizer_mass.get();
    const scalar m_rcs = state.rcs_fuel_mass.get();
    const scalar m_press = state.pressurant_mass.get();

    if (m_dry <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (m_f < scalar{0.0f} || m_o < scalar{0.0f} ||
        m_rcs < scalar{0.0f} || m_press < scalar{0.0f}) {
        APSA_UNLIKELY { return false; }
    }

    const scalar total = m_dry + m_f + m_o + m_rcs + m_press;

    if (!std::isfinite(total)) APSA_UNLIKELY {
        return false;
    }

    result = Mass{total};
    return true;
}

[[nodiscard]] constexpr bool mass_rate(
    const MassFlowRate fuel_depletion,
    const MassFlowRate oxidizer_depletion,
    const MassFlowRate rcs_depletion,
    MassFlowRate& result
) noexcept {
    const scalar m_dot_f = fuel_depletion.get();
    const scalar m_dot_o = oxidizer_depletion.get();
    const scalar m_dot_rcs = rcs_depletion.get();

    const scalar total_rate = m_dot_f + m_dot_o + m_dot_rcs;

    if (!std::isfinite(total_rate)) APSA_UNLIKELY {
        return false;
    }

    result = MassFlowRate{total_rate};
    return true;
}

// ═══════════════════════════════════════════
// CG
// ═══════════════════════════════════════════

struct MassElement {
    Mass mass{};
    UnitVec3<Length> position{};
};

struct MovingMassElement {
    Mass mass{};
    MassFlowRate mass_rate{};
    UnitVec3<Length> position{};
    UnitVec3<Velocity> velocity{};
};

[[nodiscard]] constexpr bool cg_position(
    const std::array<MassElement, kMaxMassElements>& elements,
    const std::size_t element_count,
    const std::array<MassElement, kMaxSloshMasses>& slosh_elements,
    const std::size_t slosh_count,
    const Mass total_mass,
    UnitVec3<Length>& cg
) noexcept {
    const scalar m_total = total_mass.get();

    if (m_total <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (element_count == 0U || element_count > kMaxMassElements) {
        APSA_UNLIKELY { return false; }
    }

    Vec3 weighted_sum{0.0f, 0.0f, 0.0f};

    for (std::size_t i{0}; i < element_count; ++i) {
        const scalar m_i = elements[i].mass.get();
        const Vec3 r_i = elements[i].position.data;

        weighted_sum = weighted_sum + (r_i * m_i);
    }

    for (std::size_t k{0}; k < slosh_count; ++k) {
        const scalar m_k = slosh_elements[k].mass.get();
        const Vec3 r_k = slosh_elements[k].position.data;

        weighted_sum = weighted_sum + (r_k * m_k);
    }

    const scalar inv_m = scalar{1.0f} / m_total;
    const Vec3 cg_pos = weighted_sum * inv_m;

    if (!std::isfinite(cg_pos.x()) ||
        !std::isfinite(cg_pos.y()) ||
        !std::isfinite(cg_pos.z())) {
        APSA_UNLIKELY { return false; }
    }

    cg = UnitVec3<Length>{cg_pos};
    return true;
}

[[nodiscard]] constexpr bool cg_velocity(
    const std::array<MovingMassElement, kMaxMassElements>& elements,
    const std::size_t count,
    const Mass total_mass,
    const MassFlowRate total_mass_rate,
    const UnitVec3<Length>& current_cg,
    UnitVec3<Velocity>& cg_vel
) noexcept {
    const scalar m_total = total_mass.get();
    const scalar m_dot_total = total_mass_rate.get();

    if (m_total <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (count == 0U || count > kMaxMassElements) APSA_UNLIKELY {
        return false;
    }

    Vec3 sum{0.0f, 0.0f, 0.0f};

    for (std::size_t i{0}; i < count; ++i) {
        const scalar m_dot_i = elements[i].mass_rate.get();
        const scalar m_i = elements[i].mass.get();
        const Vec3 r_i = elements[i].position.data;
        const Vec3 r_dot_i = elements[i].velocity.data;

        sum = sum + (r_i * m_dot_i) + (r_dot_i * m_i);
    }

    const scalar inv_m = scalar{1.0f} / m_total;
    const Vec3 cg_velocity = (sum * inv_m) -
        (current_cg.data * (m_dot_total * inv_m));

    if (!std::isfinite(cg_velocity.x()) ||
        !std::isfinite(cg_velocity.y()) ||
        !std::isfinite(cg_velocity.z())) {
        APSA_UNLIKELY { return false; }
    }

    cg_vel = UnitVec3<Velocity>{cg_velocity};
    return true;
}

[[nodiscard]] constexpr bool cg_shift_fuel(
    const Mass old_mass,
    const Mass new_mass,
    const UnitVec3<Length>& old_cg,
    const Mass delta_fuel_mass,
    const UnitVec3<Length>& fuel_centroid,
    UnitVec3<Length>& new_cg
) noexcept {
    const scalar m_old = old_mass.get();
    const scalar m_new = new_mass.get();
    const scalar dm_f = delta_fuel_mass.get();

    if (m_old <= kNormSqThreshold || m_new <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }
    if (dm_f < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const Vec3 numerator = old_cg.data * m_old +
        fuel_centroid.data * dm_f;

    const scalar inv_m_new = scalar{1.0f} / m_new;
    const Vec3 r_cg_new = numerator * inv_m_new;

    if (!std::isfinite(r_cg_new.x()) ||
        !std::isfinite(r_cg_new.y()) ||
        !std::isfinite(r_cg_new.z())) {
        APSA_UNLIKELY { return false; }
    }

    new_cg = UnitVec3<Length>{r_cg_new};
    return true;
}

[[nodiscard]] constexpr bool cg_shift_oxidizer(
    const Mass old_mass,
    const Mass new_mass,
    const UnitVec3<Length>& old_cg,
    const Mass delta_oxidizer_mass,
    const UnitVec3<Length>& oxidizer_centroid,
    UnitVec3<Length>& new_cg
) noexcept {
    const scalar m_old = old_mass.get();
    const scalar m_new = new_mass.get();
    const scalar dm_o = delta_oxidizer_mass.get();

    if (m_old <= kNormSqThreshold || m_new <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }
    if (dm_o < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const Vec3 numerator = old_cg.data * m_old +
        oxidizer_centroid.data * dm_o;

    const scalar inv_m_new = scalar{1.0f} / m_new;
    const Vec3 r_cg_new = numerator * inv_m_new;

    if (!std::isfinite(r_cg_new.x()) ||
        !std::isfinite(r_cg_new.y()) ||
        !std::isfinite(r_cg_new.z())) {
        APSA_UNLIKELY { return false; }
    }

    new_cg = UnitVec3<Length>{r_cg_new};
    return true;
}

// ═══════════════════════════════════════════
// INERTIA TENSOR
// ═══════════════════════════════════════════

struct InertiaTensorPhysics {
    scalar Ixx{0.0f};
    scalar Iyy{0.0f};
    scalar Izz{0.0f};
    scalar Ixy{0.0f};
    scalar Ixz{0.0f};
    scalar Iyz{0.0f};
    bool valid{false};
};

[[nodiscard]] constexpr bool inertia_build_matrix(
    const InertiaTensorPhysics& I,
    Mat3& I_matrix
) noexcept {
    I_matrix = Mat3{{
         I.Ixx, -I.Ixy, -I.Ixz,
        -I.Ixy,  I.Iyy, -I.Iyz,
        -I.Ixz, -I.Iyz,  I.Izz
    }};

    return true;
}

[[nodiscard]] constexpr bool parallel_axis_theorem(
    const Mat3& inertia_cm,
    const Mass mass,
    const UnitVec3<Length>& offset,
    Mat3& inertia_offset
) noexcept {
    const scalar m = mass.get();
    const Vec3 d = offset.data;

    if (m <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar d_sq = dot(d, d);

    const Mat3 outer = Mat3{{
        d.x()*d.x(), d.x()*d.y(), d.x()*d.z(),
        d.y()*d.x(), d.y()*d.y(), d.y()*d.z(),
        d.z()*d.x(), d.z()*d.y(), d.z()*d.z()
    }};

    const Mat3 correction = Mat3{{
        (d_sq - outer.at(0,0)) * m,
        (-outer.at(0,1)) * m,
        (-outer.at(0,2)) * m,
        (-outer.at(1,0)) * m,
        (d_sq - outer.at(1,1)) * m,
        (-outer.at(1,2)) * m,
        (-outer.at(2,0)) * m,
        (-outer.at(2,1)) * m,
        (d_sq - outer.at(2,2)) * m
    }};

    inertia_offset = inertia_cm + correction;

    for (std::size_t i{0}; i < 3U; ++i) {
        for (std::size_t j{0}; j < 3U; ++j) {
            if (!std::isfinite(inertia_offset.at(i, j))) APSA_UNLIKELY {
                return false;
            }
        }
    }

    return true;
}

[[nodiscard]] constexpr bool parallel_axis_full(
    const InertiaTensorPhysics& I_cm,
    const Mass mass,
    const scalar dx,
    const scalar dy,
    const scalar dz,
    InertiaTensorPhysics& I_offset
) noexcept {
    if (!I_cm.valid) APSA_UNLIKELY {
        return false;
    }

    const scalar m = mass.get();

    if (m < kPAxisMinMass) APSA_UNLIKELY {
        return false;
    }

    const scalar d_sq = dx*dx + dy*dy + dz*dz;

    I_offset.Ixx = I_cm.Ixx + m * (d_sq - dx*dx);
    I_offset.Iyy = I_cm.Iyy + m * (d_sq - dy*dy);
    I_offset.Izz = I_cm.Izz + m * (d_sq - dz*dz);

    I_offset.Ixy = I_cm.Ixy + m * dx * dy;
    I_offset.Ixz = I_cm.Ixz + m * dx * dz;
    I_offset.Iyz = I_cm.Iyz + m * dy * dz;

    if (!std::isfinite(I_offset.Ixx) ||
        !std::isfinite(I_offset.Iyy) ||
        !std::isfinite(I_offset.Izz) ||
        !std::isfinite(I_offset.Ixy) ||
        !std::isfinite(I_offset.Ixz) ||
        !std::isfinite(I_offset.Iyz)) APSA_UNLIKELY {
        return false;
    }

    if (I_offset.Ixx < kPAxisMinInertia ||
        I_offset.Iyy < kPAxisMinInertia ||
        I_offset.Izz < kPAxisMinInertia) APSA_UNLIKELY {
        return false;
    }

    I_offset.valid = true;
    return true;
}

[[nodiscard]] constexpr bool parallel_axis_reverse(
    const InertiaTensorPhysics& I_offset,
    const Mass mass,
    const scalar dx,
    const scalar dy,
    const scalar dz,
    InertiaTensorPhysics& I_cm
) noexcept {
    if (!I_offset.valid) APSA_UNLIKELY {
        return false;
    }

    const scalar m = mass.get();

    if (m < kPAxisMinMass) APSA_UNLIKELY {
        return false;
    }

    const scalar d_sq = dx*dx + dy*dy + dz*dz;

    I_cm.Ixx = I_offset.Ixx - m * (d_sq - dx*dx);
    I_cm.Iyy = I_offset.Iyy - m * (d_sq - dy*dy);
    I_cm.Izz = I_offset.Izz - m * (d_sq - dz*dz);

    I_cm.Ixy = I_offset.Ixy - m * dx * dy;
    I_cm.Ixz = I_offset.Ixz - m * dx * dz;
    I_cm.Iyz = I_offset.Iyz - m * dy * dz;

    if (!std::isfinite(I_cm.Ixx) ||
        !std::isfinite(I_cm.Iyy) ||
        !std::isfinite(I_cm.Izz)) APSA_UNLIKELY {
        return false;
    }

    if (I_cm.Ixx < kPAxisMinInertia ||
        I_cm.Iyy < kPAxisMinInertia ||
        I_cm.Izz < kPAxisMinInertia) APSA_UNLIKELY {
        return false;
    }

    I_cm.valid = true;
    return true;
}

[[nodiscard]] constexpr bool inertia_merge(
    const InertiaTensorPhysics& I1,
    const Mass m1,
    const scalar d1x, const scalar d1y, const scalar d1z,
    const InertiaTensorPhysics& I2,
    const Mass m2,
    const scalar d2x, const scalar d2y, const scalar d2z,
    InertiaTensorPhysics& I_combined
) noexcept {
    if (!I1.valid || !I2.valid) APSA_UNLIKELY {
        return false;
    }

    const scalar mass1 = m1.get();
    const scalar mass2 = m2.get();

    if (mass1 < kPAxisMinMass || mass2 < kPAxisMinMass) APSA_UNLIKELY {
        return false;
    }

    InertiaTensorPhysics I1_offset{};
    if (!parallel_axis_full(I1, m1, d1x, d1y, d1z, I1_offset)) APSA_UNLIKELY {
        return false;
    }

    InertiaTensorPhysics I2_offset{};
    if (!parallel_axis_full(I2, m2, d2x, d2y, d2z, I2_offset)) APSA_UNLIKELY {
        return false;
    }

    I_combined.Ixx = I1_offset.Ixx + I2_offset.Ixx;
    I_combined.Iyy = I1_offset.Iyy + I2_offset.Iyy;
    I_combined.Izz = I1_offset.Izz + I2_offset.Izz;
    I_combined.Ixy = I1_offset.Ixy + I2_offset.Ixy;
    I_combined.Ixz = I1_offset.Ixz + I2_offset.Ixz;
    I_combined.Iyz = I1_offset.Iyz + I2_offset.Iyz;

    if (!std::isfinite(I_combined.Ixx) ||
        !std::isfinite(I_combined.Iyy) ||
        !std::isfinite(I_combined.Izz)) APSA_UNLIKELY {
        return false;
    }

    I_combined.valid = true;
    return true;
}

[[nodiscard]] constexpr bool inertia_determinant(
    const InertiaTensorPhysics& I,
    scalar& det
) noexcept {
    const scalar term1 = I.Ixx * (
        I.Iyy * I.Izz - I.Iyz * I.Iyz
    );

    const scalar term2 = I.Ixy * (
        I.Ixy * I.Izz - I.Iyz * I.Ixz
    );

    const scalar term3 = I.Ixz * (
        I.Ixy * I.Iyz - I.Iyy * I.Ixz
    );

    det = term1 - term2 + term3;

    if (!std::isfinite(det)) APSA_UNLIKELY {
        return false;
    }

    if (std::abs(det) < kInertiaMinDet) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool inertia_time_derivative(
    const MassFlowRate m_fuel_dot,
    const MassFlowRate m_oxidizer_dot,
    const Length fuel_cg_y,
    const Length fuel_cg_z,
    const Length ox_cg_y,
    const Length ox_cg_z,
    const scalar sigma_fuel_sq,
    const scalar sigma_ox_sq,
    InertiaTensorPhysics& I_dot
) noexcept {
    const scalar dIxx_dmf =
        (fuel_cg_y.get() * fuel_cg_y.get() +
         fuel_cg_z.get() * fuel_cg_z.get()) + sigma_fuel_sq;

    const scalar dIxx_dmo =
        (ox_cg_y.get() * ox_cg_y.get() +
         ox_cg_z.get() * ox_cg_z.get()) + sigma_ox_sq;

    I_dot.Ixx = dIxx_dmf * m_fuel_dot.get() +
                dIxx_dmo * m_oxidizer_dot.get();

    const scalar dIyy_dmf =
        (fuel_cg_y.get() * fuel_cg_y.get() +
         fuel_cg_z.get() * fuel_cg_z.get()) + sigma_fuel_sq;

    const scalar dIyy_dmo =
        (ox_cg_y.get() * ox_cg_y.get() +
         ox_cg_z.get() * ox_cg_z.get()) + sigma_ox_sq;

    I_dot.Iyy = dIyy_dmf * m_fuel_dot.get() +
                dIyy_dmo * m_oxidizer_dot.get();

    const scalar dIzz_dmf =
        (fuel_cg_y.get() * fuel_cg_y.get() +
         fuel_cg_z.get() * fuel_cg_z.get()) + sigma_fuel_sq;

    const scalar dIzz_dmo =
        (ox_cg_y.get() * ox_cg_y.get() +
         ox_cg_z.get() * ox_cg_z.get()) + sigma_ox_sq;

    I_dot.Izz = dIzz_dmf * m_fuel_dot.get() +
                dIzz_dmo * m_oxidizer_dot.get();

    I_dot.Ixy = 0.0f;
    I_dot.Ixz = 0.0f;
    I_dot.Iyz = 0.0f;

    if (!std::isfinite(I_dot.Ixx) ||
        !std::isfinite(I_dot.Iyy) ||
        !std::isfinite(I_dot.Izz)) APSA_UNLIKELY {
        return false;
    }

    I_dot.valid = true;
    return true;
}

[[nodiscard]] constexpr bool inertia_update_mass_change(
    const InertiaTensorPhysics& I_old,
    const Mass m_old,
    const Mass m_new,
    InertiaTensorPhysics& I_new
) noexcept {
    if (m_old.get() < kInertiaMinMass) APSA_UNLIKELY {
        return false;
    }

    if (m_new.get() < kInertiaMinMass) APSA_UNLIKELY {
        return false;
    }

    const scalar ratio = m_new.get() / m_old.get();

    I_new.Ixx = I_old.Ixx * ratio;
    I_new.Iyy = I_old.Iyy * ratio;
    I_new.Izz = I_old.Izz * ratio;
    I_new.Ixy = I_old.Ixy * ratio;
    I_new.Ixz = I_old.Ixz * ratio;
    I_new.Iyz = I_old.Iyz * ratio;

    if (!std::isfinite(I_new.Ixx) ||
        !std::isfinite(I_new.Iyy) ||
        !std::isfinite(I_new.Izz)) APSA_UNLIKELY {
        return false;
    }

    I_new.valid = true;
    return true;
}

[[nodiscard]] constexpr bool inertia_validate_physical(
    const InertiaTensorPhysics& I
) noexcept {
    if (I.Ixx < kInertiaMinValue) APSA_UNLIKELY {
        return false;
    }

    if (I.Iyy < kInertiaMinValue) APSA_UNLIKELY {
        return false;
    }

    if (I.Izz < kInertiaMinValue) APSA_UNLIKELY {
        return false;
    }

    if (I.Ixx + I.Iyy < I.Izz - kInertiaMinValue) APSA_UNLIKELY {
        return false;
    }

    if (I.Iyy + I.Izz < I.Ixx - kInertiaMinValue) APSA_UNLIKELY {
        return false;
    }

    if (I.Ixx + I.Izz < I.Iyy - kInertiaMinValue) APSA_UNLIKELY {
        return false;
    }

    return true;
}

[[nodiscard]] constexpr bool inertia_full_update(
    const InertiaTensorPhysics& I_current,
    const InertiaTensorPhysics& I_dot,
    const Time dt,
    InertiaTensorPhysics& I_new
) noexcept {
    if (dt.get() <= scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar h = dt.get();

    I_new.Ixx = I_current.Ixx + h * I_dot.Ixx;
    I_new.Iyy = I_current.Iyy + h * I_dot.Iyy;
    I_new.Izz = I_current.Izz + h * I_dot.Izz;
    I_new.Ixy = I_current.Ixy + h * I_dot.Ixy;
    I_new.Ixz = I_current.Ixz + h * I_dot.Ixz;
    I_new.Iyz = I_current.Iyz + h * I_dot.Iyz;

    if (!inertia_validate_physical(I_new)) APSA_UNLIKELY {
        return false;
    }

    I_new.valid = true;
    return true;
}

// ═══════════════════════════════════════════
// FUEL DEPLETION + TANK
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool fuel_depletion(
    const Force thrust,
    const SpecificImpulse specific_impulse,
    const Dimensionless mixture_ratio,
    const Dimensionless utilization_efficiency,
    MassFlowRate& result
) noexcept {
    const scalar T = thrust.get();
    const scalar I_sp = specific_impulse.get();
    const scalar MR = mixture_ratio.get();
    const scalar eta = utilization_efficiency.get();

    if (T < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    if (I_sp <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (MR < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    if (eta <= kNormSqThreshold || eta > scalar{1.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar denom = I_sp * kGravity * (scalar{1.0f} + MR) * eta;
    const scalar m_dot = -T / denom;

    if (!std::isfinite(m_dot)) APSA_UNLIKELY {
        return false;
    }

    result = MassFlowRate{m_dot};
    return true;
}

[[nodiscard]] constexpr bool oxidizer_depletion(
    const MassFlowRate fuel_depletion_rate,
    const Dimensionless mixture_ratio,
    MassFlowRate& result
) noexcept {
    const scalar m_dot_f = fuel_depletion_rate.get();
    const scalar MR = mixture_ratio.get();

    if (m_dot_f > scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }
    if (MR < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    const scalar m_dot_o = -MR * m_dot_f;

    if (!std::isfinite(m_dot_o)) APSA_UNLIKELY {
        return false;
    }

    result = MassFlowRate{m_dot_o};
    return true;
}

[[nodiscard]] constexpr bool rcs_fuel_depletion(
    const std::array<Force, kMaxRcsThrusters>& rcs_forces,
    const std::array<Dimensionless, kMaxRcsThrusters>& duty_cycles,
    const std::size_t count,
    const SpecificImpulse rcs_isp,
    MassFlowRate& result
) noexcept {
    if (count == 0U || count > kMaxRcsThrusters) APSA_UNLIKELY {
        return false;
    }

    const scalar I_sp = rcs_isp.get();
    if (I_sp <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    scalar sum{0.0f};
    for (std::size_t i{0}; i < count; ++i) {
        const scalar F_i = rcs_forces[i].get();
        const scalar d_i = duty_cycles[i].get();

        if (F_i < scalar{0.0f} || d_i < scalar{0.0f} || d_i > scalar{1.0f}) {
            APSA_UNLIKELY { return false; }
        }

        sum += F_i * d_i;
    }

    const scalar denom = I_sp * kGravity;
    const scalar m_dot = -sum / denom;

    if (!std::isfinite(m_dot)) APSA_UNLIKELY {
        return false;
    }

    result = MassFlowRate{m_dot};
    return true;
}

[[nodiscard]] constexpr bool tank_pressure_blowdown(
    const Pressure initial_pressure,
    const Volume initial_volume,
    const Volume current_volume,
    const scalar gamma,
    Pressure& result
) noexcept {
    const scalar P_0 = initial_pressure.get();
    const scalar V_0 = initial_volume.get();
    const scalar V_t = current_volume.get();

    if (P_0 <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (V_0 <= kNormSqThreshold || V_t <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }
    if (gamma <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar ratio = V_0 / V_t;
    const scalar P_t = P_0 * std::exp(gamma * std::log(ratio));

    if (!std::isfinite(P_t)) APSA_UNLIKELY {
        return false;
    }

    result = Pressure{P_t};
    return true;
}

[[nodiscard]] constexpr bool pressurant_mass(
    const Pressure tank_pressure,
    const Volume tank_volume,
    const Temperature tank_temp,
    const scalar gas_constant,
    Mass& result
) noexcept {
    const scalar P = tank_pressure.get();
    const scalar V = tank_volume.get();
    const scalar T = tank_temp.get();
    const scalar R = gas_constant;

    if (P <= kNormSqThreshold || V <= kNormSqThreshold ||
        T <= kNormSqThreshold || R <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }

    const scalar denom = R * T;
    const scalar m = (P * V) / denom;

    if (!std::isfinite(m)) APSA_UNLIKELY {
        return false;
    }

    result = Mass{m};
    return true;
}

// ═══════════════════════════════════════════
// SPECIFIC FORCE + GYROSCOPIC
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool specific_force(
    const UnitVec3<Acceleration>& body_accel,
    const UnitVec3<AngularVel>& angular_velocity,
    const UnitVec3<Velocity>& velocity_body,
    const UnitVec3<Acceleration>& gravity_body,
    UnitVec3<Acceleration>& specific_force_result
) noexcept {
    const auto coriolis = cross(angular_velocity, velocity_body);

    const Vec3 f_imu = body_accel.data + coriolis.data - gravity_body.data;

    if (!std::isfinite(f_imu.x()) ||
        !std::isfinite(f_imu.y()) ||
        !std::isfinite(f_imu.z())) {
        APSA_UNLIKELY { return false; }
    }

    specific_force_result = UnitVec3<Acceleration>{f_imu};
    return true;
}

[[nodiscard]] constexpr bool gyroscopic_term(
    const Mat3& inertia,
    const UnitVec3<AngularVel>& angular_velocity,
    UnitVec3<Torque>& gyro_torque
) noexcept {
    const Vec3 w = angular_velocity.data;
    const Vec3 I_w = inertia * w;
    const Vec3 gyro = cross(w, I_w);

    const Vec3 M_gyro = gyro * scalar{-1.0f};

    if (!std::isfinite(M_gyro.x()) ||
        !std::isfinite(M_gyro.y()) ||
        !std::isfinite(M_gyro.z())) {
        APSA_UNLIKELY { return false; }
    }

    gyro_torque = UnitVec3<Torque>{M_gyro};
    return true;
}

[[nodiscard]] constexpr bool linear_acceleration_body(
    const Mass mass,
    const UnitVec3<Force>& non_grav_force,
    UnitVec3<Acceleration>& body_accel
) noexcept {
    const scalar m = mass.get();

    if (m <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar inv_m = scalar{1.0f} / m;
    const Vec3 accel = non_grav_force.data * inv_m;

    if (!std::isfinite(accel.x()) ||
        !std::isfinite(accel.y()) ||
        !std::isfinite(accel.z())) {
        APSA_UNLIKELY { return false; }
    }

    body_accel = UnitVec3<Acceleration>{accel};
    return true;
}

[[nodiscard]] constexpr bool linear_acceleration_inertial(
    const UnitVec3<Acceleration>& body_accel,
    const Mat3& rotation_body_to_inertial,
    const UnitVec3<Acceleration>& gravity_inertial,
    UnitVec3<Acceleration>& inertial_accel
) noexcept {
    const Vec3 a_body_rot = rotation_body_to_inertial * body_accel.data;
    const Vec3 a_total = a_body_rot + gravity_inertial.data;

    if (!std::isfinite(a_total.x()) ||
        !std::isfinite(a_total.y()) ||
        !std::isfinite(a_total.z())) {
        APSA_UNLIKELY { return false; }
    }

    inertial_accel = UnitVec3<Acceleration>{a_total};
    return true;
}

// ═══════════════════════════════════════════
// ANGULAR ACCELERATION
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool angular_acceleration(
    const Mat3& inertia,
    const Mat3& inertia_dot,
    const UnitVec3<AngularVel>& angular_velocity,
    const UnitVec3<Torque>& total_torque,
    UnitVec3<AngularAccel>& angular_accel
) noexcept {
    const scalar det = determinant(inertia);

    if (std::abs(det) < kDetThreshold) APSA_UNLIKELY {
        return false;
    }

    Mat3 inv_inertia{};
    const bool inv_ok = inverse(inertia, inv_inertia);
    if (!inv_ok) APSA_UNLIKELY {
        return false;
    }

    const Vec3 w = angular_velocity.data;
    const Vec3 I_w = inertia * w;
    const Vec3 gyro = cross(w, I_w);
    const Vec3 I_dot_w = inertia_dot * w;

    const Vec3 net = total_torque.data - gyro - I_dot_w;
    const Vec3 alpha = inv_inertia * net;

    if (!std::isfinite(alpha.x()) ||
        !std::isfinite(alpha.y()) ||
        !std::isfinite(alpha.z())) {
        APSA_UNLIKELY { return false; }
    }

    angular_accel = UnitVec3<AngularAccel>{alpha};
    return true;
}

// ═══════════════════════════════════════════
// EULER KINEMATICS 3-2-1
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool euler_kinematics_321(
    const Radian roll,
    const Radian pitch,
    const UnitVec3<AngularVel>& omega,
    Radian& roll_dot,
    Radian& pitch_dot,
    Radian& yaw_dot
) noexcept {
    const scalar phi = roll.get();
    const scalar theta = pitch.get();
    const scalar wx = omega.x();
    const scalar wy = omega.y();
    const scalar wz = omega.z();

    const scalar cos_theta = std::cos(theta);

    if (std::abs(cos_theta) < kEulerSingularityThreshold) {
        APSA_UNLIKELY { return false; }
    }

    const scalar sin_phi = std::sin(phi);
    const scalar cos_phi = std::cos(phi);
    const scalar sin_theta = std::sin(theta);

    const scalar inv_cos_theta = scalar{1.0f} / cos_theta;

    const scalar phi_dot = (wx + sin_phi * sin_theta * wy +
        cos_phi * sin_theta * wz) * inv_cos_theta;

    const scalar theta_dot = cos_phi * wy - sin_phi * wz;

    const scalar psi_dot = (sin_phi * wy + cos_phi * wz) * inv_cos_theta;

    if (!std::isfinite(phi_dot) ||
        !std::isfinite(theta_dot) ||
        !std::isfinite(psi_dot)) {
        APSA_UNLIKELY { return false; }
    }

    roll_dot = Radian{phi_dot};
    pitch_dot = Radian{theta_dot};
    yaw_dot = Radian{psi_dot};

    return true;
}

// ═══════════════════════════════════════════
// GIMBAL + MISALIGNMENT
// ═══════════════════════════════════════════

[[nodiscard]] constexpr bool gimbal_thrust_direction(
    const Radian gimbal_yaw,
    const Radian gimbal_pitch,
    Vec3& thrust_direction
) noexcept {
    const scalar cy = std::cos(gimbal_yaw.get());
    const scalar cp = std::cos(gimbal_pitch.get());
    const scalar sy = std::sin(gimbal_yaw.get());
    const scalar sp = std::sin(gimbal_pitch.get());

    thrust_direction = Vec3{
        cy * cp,
        sy,
        -cy * sp
    };

    return true;
}

[[nodiscard]] constexpr bool gimbal_torque(
    const UnitVec3<Length>& engine_position,
    const UnitVec3<Length>& cg_position,
    const Force thrust_magnitude,
    const Vec3& thrust_direction,
    UnitVec3<Torque>& gimbal_torque_result
) noexcept {
    const Vec3 r = engine_position.data - cg_position.data;
    const Vec3 thrust_vector = thrust_direction * thrust_magnitude.get();

    const Vec3 torque = cross(r, thrust_vector);

    if (!std::isfinite(torque.x()) ||
        !std::isfinite(torque.y()) ||
        !std::isfinite(torque.z())) {
        APSA_UNLIKELY { return false; }
    }

    gimbal_torque_result = UnitVec3<Torque>{torque};
    return true;
}

[[nodiscard]] constexpr bool cg_offset_torque(
    const UnitVec3<Length>& thrust_reference_position,
    const UnitVec3<Length>& current_cg_position,
    const UnitVec3<Force>& thrust_force,
    UnitVec3<Torque>& offset_torque
) noexcept {
    const Vec3 r = thrust_reference_position.data - current_cg_position.data;
    const Vec3 torque = cross(r, thrust_force.data);

    if (!std::isfinite(torque.x()) ||
        !std::isfinite(torque.y()) ||
        !std::isfinite(torque.z())) {
        APSA_UNLIKELY { return false; }
    }

    offset_torque = UnitVec3<Torque>{torque};
    return true;
}

[[nodiscard]] constexpr bool misalignment_rotation(
    const Radian epsilon_y,
    const Radian epsilon_z,
    Mat3& rotation_mis
) noexcept {
    const scalar ey = epsilon_y.get();
    const scalar ez = epsilon_z.get();

    if (std::abs(ey) > scalar{0.1f} || std::abs(ez) > scalar{0.1f}) {
        APSA_UNLIKELY { return false; }
    }

    const Mat3 R2_y = r2(epsilon_y);
    const Mat3 R3_z = r3(epsilon_z);

    rotation_mis = R2_y * R3_z;

    return true;
}

[[nodiscard]] constexpr bool misalignment_torque(
    const UnitVec3<Length>& thrust_reference_position,
    const UnitVec3<Length>& cg_position,
    const Force thrust_magnitude,
    const Mat3& rotation_mis,
    UnitVec3<Torque>& mis_torque
) noexcept {
    const Vec3 x_axis{1.0f, 0.0f, 0.0f};
    const Vec3 thrust_direction = rotation_mis * x_axis;

    const Vec3 r = thrust_reference_position.data - cg_position.data;
    const Vec3 thrust_vector = thrust_direction * thrust_magnitude.get();

    const Vec3 torque = cross(r, thrust_vector);

    if (!std::isfinite(torque.x()) ||
        !std::isfinite(torque.y()) ||
        !std::isfinite(torque.z())) {
        APSA_UNLIKELY { return false; }
    }

    mis_torque = UnitVec3<Torque>{torque};
    return true;
}

// ═══════════════════════════════════════════
// SLOSH DYNAMICS
// ═══════════════════════════════════════════

struct SloshMode {
    Mass slosh_mass{};
    Length pendulum_length{};
    scalar damping{0.0f};
    Radian angle{0.0f};
    AngularVel angle_rate{0.0f};
    UnitVec3<Length> mode_direction{};
};

[[nodiscard]] constexpr bool slosh_frequency(
    const Length tank_radius,
    const Length fill_height,
    const Acceleration effective_gravity,
    AngularVel& frequency
) noexcept {
    const scalar R = tank_radius.get();
    const scalar h = fill_height.get();
    const scalar g = effective_gravity.get();

    if (R <= kNormSqThreshold || h <= kNormSqThreshold ||
        g <= kNormSqThreshold) {
        APSA_UNLIKELY { return false; }
    }

    const scalar arg = scalar{1.841f} * h / R;
    const scalar tanh_val = std::tanh(arg);
    const scalar omega_sq = (scalar{1.841f} * g / R) * tanh_val;

    if (omega_sq <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    frequency = AngularVel{std::sqrt(omega_sq)};
    return true;
}

[[nodiscard]] constexpr bool slosh_acceleration(
    const SloshMode& mode,
    const Acceleration lateral_accel,
    const Acceleration effective_gravity,
    AngularAccel& angular_accel
) noexcept {
    const scalar m = mode.slosh_mass.get();
    const scalar L = mode.pendulum_length.get();
    const scalar c = mode.damping;
    const scalar g = effective_gravity.get();

    if (L <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar phi = mode.angle.get();
    const scalar phi_dot = mode.angle_rate.get();

    const scalar inertia = m * L * L;
    const scalar gravity_term = m * L * g * std::cos(phi);
    const scalar damping_term = c * phi_dot;
    const scalar forcing = m * L * lateral_accel.get();

    const scalar phi_ddot = (-damping_term - gravity_term - forcing) / inertia;

    if (!std::isfinite(phi_ddot)) APSA_UNLIKELY {
        return false;
    }

    angular_accel = AngularAccel{phi_ddot};
    return true;
}

[[nodiscard]] constexpr bool slosh_force(
    const std::array<SloshMode, kMaxSloshModes>& modes,
    const std::size_t count,
    UnitVec3<Force>& total_slosh_force
) noexcept {
    if (count == 0U || count > kMaxSloshModes) APSA_UNLIKELY {
        return false;
    }

    Vec3 total{0.0f, 0.0f, 0.0f};

    for (std::size_t k{0}; k < count; ++k) {
        const scalar m = modes[k].slosh_mass.get();
        const scalar L = modes[k].pendulum_length.get();
        const scalar phi_dot = modes[k].angle_rate.get();

        const scalar phi_ddot_term = L * phi_dot * phi_dot;
        const scalar force_mag = m * L * phi_ddot_term;

        total = total + (modes[k].mode_direction.data * force_mag);
    }

    if (!std::isfinite(total.x()) ||
        !std::isfinite(total.y()) ||
        !std::isfinite(total.z())) {
        APSA_UNLIKELY { return false; }
    }

    total_slosh_force = UnitVec3<Force>{total};
    return true;
}

// ═══════════════════════════════════════════
// FLEXIBLE MODES
// ═══════════════════════════════════════════

struct FlexibleMode {
    scalar natural_frequency{0.0f};
    scalar damping_ratio{0.0f};
    scalar modal_displacement{0.0f};
    scalar modal_velocity{0.0f};
    Vec3 force_coupling{0.0f, 0.0f, 0.0f};
    Vec3 torque_coupling{0.0f, 0.0f, 0.0f};
    scalar linear_momentum_coeff{0.0f};
    Vec3 angular_momentum_coeff{0.0f, 0.0f, 0.0f};
};

[[nodiscard]] constexpr bool flexible_mode_acceleration(
    const FlexibleMode& mode,
    const UnitVec3<Force>& applied_force,
    const UnitVec3<AngularAccel>& applied_alpha,
    scalar& modal_accel
) noexcept {
    const scalar omega = mode.natural_frequency;
    const scalar zeta = mode.damping_ratio;
    const scalar eta = mode.modal_displacement;
    const scalar eta_dot = mode.modal_velocity;

    if (omega <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar force_term = dot(mode.force_coupling, applied_force.data);
    const scalar torque_term = dot(mode.torque_coupling, applied_alpha.data);

    const scalar omega_sq = omega * omega;
    const scalar damping = scalar{2.0f} * zeta * omega;

    const scalar eta_ddot = force_term + torque_term -
        damping * eta_dot - omega_sq * eta;

    if (!std::isfinite(eta_ddot)) APSA_UNLIKELY {
        return false;
    }

    modal_accel = eta_ddot;
    return true;
}

[[nodiscard]] constexpr bool flexible_coupled_dynamics_full(
    const Mass total_mass,
    const Mat3& total_inertia,
    const UnitVec3<Force>& applied_force,
    const UnitVec3<Torque>& applied_torque,
    const std::array<FlexibleMode, kMaxFlexibleModes>& modes,
    const std::size_t count,
    UnitVec3<Acceleration>& linear_accel,
    UnitVec3<AngularAccel>& angular_accel
) noexcept {
    const scalar m = total_mass.get();

    if (m <= kNormSqThreshold || count > kMaxFlexibleModes) {
        APSA_UNLIKELY { return false; }
    }

    Vec3 modal_force{0.0f, 0.0f, 0.0f};
    Vec3 modal_torque{0.0f, 0.0f, 0.0f};

    for (std::size_t k{0}; k < count; ++k) {
        const scalar eta_ddot = modes[k].force_coupling.x() *
            applied_force.x() +
            modes[k].force_coupling.y() * applied_force.y() +
            modes[k].force_coupling.z() * applied_force.z() +
            modes[k].torque_coupling.x() * applied_torque.x() +
            modes[k].torque_coupling.y() * applied_torque.y() +
            modes[k].torque_coupling.z() * applied_torque.z();

        if (!std::isfinite(eta_ddot)) APSA_UNLIKELY {
            return false;
        }

        modal_force = modal_force +
            (modes[k].force_coupling * eta_ddot);
        modal_torque = modal_torque +
            (modes[k].torque_coupling * eta_ddot);
    }

    const Vec3 net_force = applied_force.data - modal_force;
    const Vec3 net_torque = applied_torque.data - modal_torque;

    const scalar inv_m = scalar{1.0f} / m;
    const Vec3 a_lin = net_force * inv_m;

    Mat3 inv_I{};
    const bool inv_ok = inverse(total_inertia, inv_I);
    if (!inv_ok) APSA_UNLIKELY { return false; }

    const Vec3 alpha = inv_I * net_torque;

    if (!std::isfinite(a_lin.x()) ||
        !std::isfinite(a_lin.y()) ||
        !std::isfinite(a_lin.z()) ||
        !std::isfinite(alpha.x()) ||
        !std::isfinite(alpha.y()) ||
        !std::isfinite(alpha.z())) {
        APSA_UNLIKELY { return false; }
    }

    linear_accel = UnitVec3<Acceleration>{a_lin};
    angular_accel = UnitVec3<AngularAccel>{alpha};
    return true;
}

// ═══════════════════════════════════════════
// RCS
// ═══════════════════════════════════════════

struct RCSThrusterConfig {
    Force nominal_thrust{};
    UnitVec3<Force> direction{};
    UnitVec3<Length> position{};
    Dimensionless efficiency{1.0f};
    Dimensionless duty_cycle{0.0f};
};

[[nodiscard]] constexpr bool rcs_thrust_force(
    const RCSThrusterConfig& thruster,
    const Pressure tank_pressure,
    const Pressure nominal_pressure,
    Dimensionless& force_scaling
) noexcept {
    (void)thruster;
    const scalar P_t = tank_pressure.get();
    const scalar P_nom = nominal_pressure.get();

    if (P_nom <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }
    if (P_t < scalar{0.0f}) APSA_UNLIKELY {
        return false;
    }

    force_scaling = Dimensionless{std::sqrt(P_t / P_nom)};
    return true;
}

[[nodiscard]] constexpr bool rcs_torque_matrix(
    const std::array<RCSThrusterConfig, kMaxRcsThrusters>& thrusters,
    const std::size_t count,
    const UnitVec3<Length>& cg_position,
    Matrix_Dynamic<3, kMaxRcsThrusters>& torque_matrix
) noexcept {
    if (count == 0U || count > kMaxRcsThrusters) APSA_UNLIKELY {
        return false;
    }

    torque_matrix = Matrix_Dynamic<3, kMaxRcsThrusters>{3U, count};

    for (std::size_t i{0}; i < count; ++i) {
        const Vec3 r = thrusters[i].position.data - cg_position.data;
        const Vec3 F_dir = thrusters[i].direction.data;
        const Vec3 torque = cross(r, F_dir);

        torque_matrix.at(0, i) = torque.x();
        torque_matrix.at(1, i) = torque.y();
        torque_matrix.at(2, i) = torque.z();
    }

    return true;
}

[[nodiscard]] bool rcs_allocation(
    const Matrix_Dynamic<3, kMaxRcsThrusters>& torque_matrix,
    const UnitVec3<Torque>& commanded_torque,
    Vector_Dynamic<kMaxRcsThrusters>& thruster_commands
) noexcept {
    const std::size_t n = torque_matrix.cols();

    if (n == 0U) APSA_UNLIKELY {
        return false;
    }

    const auto B_T = transpose_Dynamic(torque_matrix);
    const auto B_B_T = torque_matrix * B_T;

    Matrix_Dynamic<3, 3> inv_B_B_T{};
    const bool inv_ok = inverse_dynamic(B_B_T, inv_B_B_T);
    if (!inv_ok) APSA_UNLIKELY {
        return false;
    }

    const Vec3 m_cmd = commanded_torque.data;
    Vector_Dynamic<3> m_vec{
        Vector_Dynamic<3>{3U}
    };
    m_vec.at(0) = m_cmd.x();
    m_vec.at(1) = m_cmd.y();
    m_vec.at(2) = m_cmd.z();

    const auto temp = inv_B_B_T * m_vec;
    thruster_commands = B_T * temp;

    for (std::size_t i{0}; i < n; ++i) {
        if (!std::isfinite(thruster_commands.at(i))) APSA_UNLIKELY {
            return false;
        }
    }

    return true;
}

// ═══════════════════════════════════════════
// SIXDOF DERIVATIVE — Full
// ═══════════════════════════════════════════

[[nodiscard]] bool sixdof_derivative(
    const Mass mass,
    const MassFlowRate m_dot,
    const InertiaTensorPhysics& I,
    const InertiaTensorPhysics& I_dot,
    const UnitVec3<Force>& F_total,
    const UnitVec3<Torque>& M_total,
    const UnitVec3<Velocity>& v,
    const AngularVel omega_x,
    const AngularVel omega_y,
    const AngularVel omega_z,
    SixDofDerivative& deriv
) noexcept {
    (void)m_dot;
    if (!sixdof_translation_body(mass, F_total,
                                 omega_x, omega_y, omega_z,
                                 v, deriv.v_dot)) {
        APSA_UNLIKELY { return false; }
    }

    Mat3 I_mat{};
    if (!inertia_build_matrix(I, I_mat)) APSA_UNLIKELY {
        return false;
    }

    Mat3 I_dot_mat{};
    if (!inertia_build_matrix(I_dot, I_dot_mat)) APSA_UNLIKELY {
        return false;
    }

    UnitVec3<AngularVel> omega_vec{Vec3{omega_x.get(), omega_y.get(), omega_z.get()}};

    UnitVec3<AngularAccel> alpha_vec{};
    if (!newton_euler_rotation(I_mat, I_dot_mat,
                                omega_vec, M_total, alpha_vec)) {
        APSA_UNLIKELY { return false; }
    }

    deriv.alpha_x = AngularVel{alpha_vec.x()};
    deriv.alpha_y = AngularVel{alpha_vec.y()};
    deriv.alpha_z = AngularVel{alpha_vec.z()};

    return true;
}