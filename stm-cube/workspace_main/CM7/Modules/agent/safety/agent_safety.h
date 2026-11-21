#ifndef AGENT_SAFETY_H
#define AGENT_SAFETY_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT SAFETY / ENVELOPE — FROZEN
 * ============================================================
 * Physical envelope checks, self-test, escalation.
 * This is the LAST LINE OF DEFENSE. Never bypass.
 * WCET: 100 µs max.
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  agent_safety_init(void);

/* ---------- Envelope Check (before every action) ---------- */
bool            agent_safety_check(void);

/* ---------- Self Test (called at BOOT) ---------- */
agent_status_t  agent_safety_self_test(void);

/* ---------- Escalation (force EMERGENCY) ---------- */
void            agent_safety_escalate(void);

/* ---------- Landability Check ---------- */
bool            agent_safety_can_land(void);

/* ---------- Landed Check ---------- */
bool            agent_safety_is_landed(void);

/* ---------- Envelope Limit Query ---------- */
float           agent_safety_envelope_accel_g(void);
float           agent_safety_envelope_gyro_dps(void);
float           agent_safety_envelope_alt_max_m(void);
float           agent_safety_envelope_vel_max_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_SAFETY_H */