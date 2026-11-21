/* ============================================================
 * APSA MODULES — COMPILE UNIT
 * ============================================================
 * This file includes all APSA module headers to force
 * compilation into object code.
 *
 * DO NOT add logic here. This is only a compilation trigger.
 * All headers are header-only (C++).
 * ============================================================ */

/* --- Math Core --- */
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "math_assert.hpp"
#include "math_attributes.hpp"

/* --- Physics --- */
#include "apsa_physics_math.hpp"
#include "apsa_physics_gravity.hpp"
#include "physics_rigid_body.hpp"
#include "apsa_frame_system.hpp"

/* --- Estimation --- */
#include "estimation_types.hpp"
#include "estimation_eskf.hpp"
#include "estimation_ekf.hpp"
#include "estimation_ukf.hpp"
#include "estimation_innovation_monitor.hpp"
#include "estimation_sensor_delay.hpp"

/* --- Control --- */
#include "control_allocation.hpp"
#include "control_bumpless.hpp"
#include "control_lqr.hpp"
#include "control_mpc.hpp"
#include "control_smc.hpp"
#include "control_sliding_extended.hpp"

/* --- Config stubs --- */
#include "apsa_physics_config.hpp"
#include "apsa_estimation_config.hpp"
#include "apsa_control_config.hpp"

/* ============================================================
 * This translation unit has no runtime logic.
 * Its only purpose is to compile all headers.
 * ============================================================ */
