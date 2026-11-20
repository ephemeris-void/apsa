/* ============================================================
 * APSA MODULES — COMPILE UNIT (DRONE ONLY)
 * ============================================================
 * Rocket-specific headers (gravity, rigid_body, frame_system,
 * physics_math, ukf, sensor_delay, sliding_extended) are
 * kept in ~/apsa_final/src/verified_logic/apsa_modules/
 * for future use. Not compiled here.
 * ============================================================ */

/* --- Math Core --- */
#include "apsa_math_engine.hpp"
#include "apsa_unit_system.hpp"
#include "apsa_frame_system.hpp"
#include "math_assert.hpp"
#include "math_attributes.hpp"

/* --- Estimation --- */
#include "estimation_types.hpp"
#include "estimation_eskf.hpp"
#include "estimation_ekf.hpp"
#include "estimation_innovation_monitor.hpp"

/* --- Control --- */
#include "control_bumpless.hpp"
#include "control_lqr.hpp"
#include "control_smc.hpp"
#include "control_mpc.hpp"
#include "control_allocation.hpp"

/* --- Config stubs --- */
#include "apsa_physics_config.hpp"
#include "apsa_estimation_config.hpp"
#include "apsa_control_config.hpp"

/* ============================================================
 * This translation unit has no runtime logic.
 * ============================================================ */
