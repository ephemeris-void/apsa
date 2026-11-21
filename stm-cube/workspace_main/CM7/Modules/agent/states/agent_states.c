#include "main.h"
#include <stdio.h>
#include "agent_states.h"
#include "agent_transitions.h"
#include "../decisions/agent_decisions.h"
#include "../actions/agent_actions.h"
#include "../actions/agent_actions_ai.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_control_if.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT STATES — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Each state handler: WCET 200 µs max
 * ============================================================ */

/* ---------- STATE LIFECYCLE ---------- */
void agent_state_set(agent_state_t new_state)
{
    agent_context_full_t *ctx = agent_context_get();
    ctx->previous_state = ctx->current_state;
    ctx->current_state  = new_state;
}

void agent_state_enter(agent_state_t state)
{
    agent_context_full_t *ctx = agent_context_get();
    ctx->state_entry_us   = agent_time_now_us();
    ctx->time_in_state_us = 0U;

    agent_trace_state_enter(state);
    agent_log_event(AGENT_LOG_STATE_ENTER, (uint32_t)state, 0U);

    switch (state) {
        case AGENT_STATE_BOOT:
            break;

        case AGENT_STATE_MONITOR:
            agent_action_set_mode(AGENT_MODE_NORMAL);
            break;

        case AGENT_STATE_DEGRADED:
            agent_action_set_mode(AGENT_MODE_DEGRADED);
            break;

        case AGENT_STATE_VIRTUALIZE:
            agent_action_set_mode(AGENT_MODE_VIRTUAL);
            agent_action_ai_trigger();
            break;

        case AGENT_STATE_FUSION:
            agent_action_set_mode(AGENT_MODE_FUSION);
            break;

        case AGENT_STATE_RECOVERY:
            agent_action_set_mode(AGENT_MODE_RECOVERY);
            break;

        case AGENT_STATE_EMERGENCY:
            agent_action_set_mode(AGENT_MODE_EMERGENCY);
            agent_action_emergency();
            break;

        case AGENT_STATE_LANDING:
            agent_action_set_mode(AGENT_MODE_LANDING);
            break;

        default:
            break;
    }
}

void agent_state_exit(agent_state_t state)
{
    agent_log_event(AGENT_LOG_STATE_EXIT,
                    (uint32_t)state,
                    agent_time_in_state_us());
}

void agent_state_run(agent_state_t state)
{
    agent_trace_state_begin(state);

    switch (state) {
        case AGENT_STATE_BOOT:       agent_state_boot();       break;
        case AGENT_STATE_MONITOR:    agent_state_monitor();    break;
        case AGENT_STATE_DEGRADED:   agent_state_degraded();   break;
        case AGENT_STATE_VIRTUALIZE: agent_state_virtualize(); break;
        case AGENT_STATE_FUSION:     agent_state_fusion();     break;
        case AGENT_STATE_RECOVERY:   agent_state_recovery();   break;
        case AGENT_STATE_EMERGENCY:  agent_state_emergency();  break;
        case AGENT_STATE_LANDING:    agent_state_landing();    break;
        default: break;
    }

    agent_trace_state_end(state);
}

/* ---------- BOOT STATE ---------- */
void agent_state_boot(void)
{
    if (agent_safety_self_test() != AGENT_OK) {
        agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_NONE);
        return;
    }

    /* Probe IMU hardware directly */
    if (imu_get_sample(NULL) == AGENT_OK) {
        agent_health_set_imu(AGENT_HEALTH_OK);
    } else {
        agent_health_set_imu(AGENT_HEALTH_FAIL);
    }

    /* Probe AI */
    if (ai_health() == AGENT_OK) {
        agent_health_set_ai(AGENT_HEALTH_OK);
    } else {
        agent_health_set_ai(AGENT_HEALTH_FAIL);
    }

    /* EKF — do NOT force IMU fail on EKF issue */
    if (ekf_health() != AGENT_OK) {
        agent_log_event(AGENT_LOG_EKF_DIVERGE, 0U, 0U);
    }

    agent_transition_to(AGENT_STATE_MONITOR, AGENT_REASON_NONE);
}

/* ---------- MONITOR STATE ---------- */
void agent_state_monitor(void)
{
    agent_sensor_sample_t sample;
    agent_context_full_t *ctx = agent_context_get();

    if (imu_get_sample(&sample) == AGENT_OK &&
        imu_health() == AGENT_HEALTH_OK) {

        ekf_update(&sample);

        /* Track consecutive OKs */
        ctx->imu_consecutive_ok++;
        ctx->imu_consecutive_fail = 0U;

    } else {
        /* IMU failed or degraded */
        ctx->imu_consecutive_fail++;
        ctx->imu_consecutive_ok = 0U;

        if (imu_health() == AGENT_HEALTH_FAIL) {
            /* Hard fail → VIRTUALIZE */
            agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_TIMEOUT);
            return;
        } else {
            /* Soft fail → DEGRADED */
            agent_transition_to(AGENT_STATE_DEGRADED, AGENT_REASON_DRIFT);
            return;
        }
    }

    /* Update control */
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) {
        control_update(&state);
    }
}

/* ---------- DEGRADED STATE ---------- */
void agent_state_degraded(void)
{
    agent_sensor_sample_t sample;
    agent_context_full_t *ctx = agent_context_get();

    if (imu_get_sample(&sample) == AGENT_OK &&
        imu_health() == AGENT_HEALTH_OK) {

        ctx->imu_consecutive_ok++;
        ctx->imu_consecutive_fail = 0U;

        /* Recovery confirmed → RECOVERY */
        if (ctx->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_RECOVERY, AGENT_REASON_NONE);
            return;
        }

        /* Feed EKF with real data */
        ekf_update(&sample);

    } else {
        ctx->imu_consecutive_fail++;
        ctx->imu_consecutive_ok = 0U;

        /* Confirmed fail → VIRTUALIZE */
        if (ctx->imu_consecutive_fail >= AGENT_FAIL_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_INNOVATION);
            return;
        }
    }

    /* Update control with last known state */
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) {
        control_update(&state);
    }
}

/* ---------- VIRTUALIZE STATE ---------- */
void agent_state_virtualize(void)
{
    agent_context_full_t *ctx = agent_context_get();

    /* SAFETY: check for IMU recovery FIRST.
     * If IMU came back (10+ consecutive OK), skip AI and go RECOVERY.
     */
    agent_sensor_sample_t real_check;
    if (imu_get_sample(&real_check) == AGENT_OK &&
        imu_health() == AGENT_HEALTH_OK) {

        ctx->imu_consecutive_ok++;
        ctx->imu_consecutive_fail = 0U;

        if (ctx->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
            ekf_update(&real_check);
            agent_recovery_register();
            agent_transition_to(AGENT_STATE_RECOVERY, AGENT_REASON_NONE);
            return;
        }
    } else {
        ctx->imu_consecutive_fail++;
        ctx->imu_consecutive_ok = 0U;
    }

    /* Trigger AI if not already running */
    if (ai_status() != AI_STATUS_RUNNING &&
        ai_status() != AI_STATUS_READY) {
        agent_action_ai_trigger();
    }

    /* Read AI output if ready */
    if (ai_output_ready()) {
        agent_sensor_sample_t virtual_sample;

        if (agent_actions_ai_read_output(&virtual_sample) == AGENT_OK) {
            ekf_update(&virtual_sample);
            agent_health_set_ai(AGENT_HEALTH_OK);

            /* Confidence recovered → FUSION */
            if (agent_confidence_get() >= AGENT_CONFIDENCE_RECOVER) {
                agent_transition_to(AGENT_STATE_FUSION, AGENT_REASON_CONFIDENCE);
                return;
            }
        } else {
            agent_health_set_ai(AGENT_HEALTH_FAIL);
        }
    }

    /* Timeout → EMERGENCY */
    if (agent_time_in_state_us() > AGENT_STATE_TIMEOUT_US) {
        agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_TIMEOUT);
        return;
    }

    /* Update control with last known state */
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) {
        control_update(&state);
    }
}

/* ---------- FUSION STATE ---------- */
void agent_state_fusion(void)
{
    agent_sensor_sample_t real_sample;
    agent_sensor_sample_t virtual_sample;
    agent_sensor_sample_t fused;
    agent_context_full_t *ctx = agent_context_get();

    bool real_ok    = (imu_get_sample(&real_sample) == AGENT_OK &&
                       imu_health() == AGENT_HEALTH_OK);
    bool virtual_ok = ai_output_ready();

    if (virtual_ok) {
        (void)agent_actions_ai_read_output(&virtual_sample);
    }

    /* Trigger AI if not already running */
    if (ai_status() != AI_STATUS_RUNNING &&
        ai_status() != AI_STATUS_READY) {
        agent_action_ai_trigger();
    }

    /* Track IMU consecutive OK/fail */
    if (real_ok) {
        ctx->imu_consecutive_ok++;
        ctx->imu_consecutive_fail = 0U;
    } else {
        ctx->imu_consecutive_fail++;
        ctx->imu_consecutive_ok = 0U;
    }

    if (real_ok && virtual_ok) {
        /* Fuse both (average) */
        fused.ax = (int16_t)(((int32_t)real_sample.ax + (int32_t)virtual_sample.ax) >> 1);
        fused.ay = (int16_t)(((int32_t)real_sample.ay + (int32_t)virtual_sample.ay) >> 1);
        fused.az = (int16_t)(((int32_t)real_sample.az + (int32_t)virtual_sample.az) >> 1);
        fused.gx = (int16_t)(((int32_t)real_sample.gx + (int32_t)virtual_sample.gx) >> 1);
        fused.gy = (int16_t)(((int32_t)real_sample.gy + (int32_t)virtual_sample.gy) >> 1);
        fused.gz = (int16_t)(((int32_t)real_sample.gz + (int32_t)virtual_sample.gz) >> 1);
        fused.timestamp_us = agent_time_now_us();
        fused.health       = AGENT_HEALTH_OK;

        ekf_update(&fused);
        agent_confidence_set(agent_confidence_get() + 0.01f);

    } else if (virtual_ok) {
        /* Only virtual */
        ekf_update(&virtual_sample);
        agent_confidence_set(1.0f);

    } else if (real_ok) {
        /* Only real */
        ekf_update(&real_sample);
        agent_confidence_set(agent_confidence_get() + 0.02f);

    } else {
        /* Both invalid → slow confidence decay */
        agent_confidence_set(agent_confidence_get() - 0.005f);

        if (agent_confidence_get() < AGENT_CONFIDENCE_MIN) {
            agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_CONFIDENCE);
            return;
        }
    }

    /* SAFETY: FUSION timeout — if stuck too long without progress */
    if (agent_time_in_state_us() > 10000000U) {
        agent_status_t st = AGENT_ERR;

        if (real_ok) {
            st = agent_transition_to(AGENT_STATE_RECOVERY, AGENT_REASON_NONE);
        } else if (virtual_ok) {
            st = agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_TIMEOUT);
        } else {
            st = agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_TIMEOUT);
        }

        if (st != AGENT_OK) {
            /* Forced fallback */
            (void)agent_transition_force(AGENT_STATE_EMERGENCY,
                                         (uint32_t)AGENT_REASON_TIMEOUT);
        }
        return;
    }

    /* Update control */
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) {
        control_update(&state);
    }
}

/* ---------- RECOVERY STATE ---------- */
void agent_state_recovery(void)
{
    agent_sensor_sample_t sample;
    agent_context_full_t *ctx = agent_context_get();

    if (imu_get_sample(&sample) == AGENT_OK &&
        imu_health() == AGENT_HEALTH_OK) {

        ctx->imu_consecutive_ok++;
        ctx->imu_consecutive_fail = 0U;

        ekf_update(&sample);
        agent_confidence_set(1.0f);

        /* Wait for consecutive OK before returning to MONITOR */
        if (ctx->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
            agent_recovery_register();
            agent_transition_to(AGENT_STATE_MONITOR, AGENT_REASON_NONE);
            return;
        }

    } else {
        ctx->imu_consecutive_fail++;
        ctx->imu_consecutive_ok = 0U;

        /* Still failing → back to VIRTUALIZE */
        if (ctx->imu_consecutive_fail >= AGENT_FAIL_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_DRIFT);
            return;
        }
    }

    /* Update control with last known state */
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) {
        control_update(&state);
    }
}

/* ---------- EMERGENCY STATE ---------- */
void agent_state_emergency(void)
{
    agent_context_full_t *ctx = agent_context_get();

    agent_action_emergency_hold();

    /* SAFETY: Minimum 10 s in EMERGENCY before LANDING.
     * Allows IMU/AI to recover; avoids premature landing on transient fault. */
    if (agent_time_in_state_us() < 10000000U) {

        /* Poll IMU to keep consecutive counter alive during EMERGENCY */
        agent_sensor_sample_t emg_sample;
        if (imu_get_sample(&emg_sample) == AGENT_OK &&
            imu_health() == AGENT_HEALTH_OK) {
            ctx->imu_consecutive_ok++;
            ctx->imu_consecutive_fail = 0U;
        } else {
            ctx->imu_consecutive_fail++;
            ctx->imu_consecutive_ok = 0U;
        }

        /* Try recovery ONLY if IMU is stably OK */
        if (imu_health() == AGENT_HEALTH_OK &&
            ctx->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_RECOVERY, AGENT_REASON_NONE);
        }
        return;
    }

    /* After 10 s minimum dwell, check if safe to land */
    if (agent_safety_can_land()) {
        agent_transition_to(AGENT_STATE_LANDING, AGENT_REASON_ENVELOPE);
        return;
    }

    /* Hard timeout */
    if (agent_time_in_state_us() > AGENT_EMERGENCY_TIMEOUT_US) {
        agent_transition_to(AGENT_STATE_LANDING, AGENT_REASON_TIMEOUT);
    }
}

/* ---------- LANDING STATE ---------- */
void agent_state_landing(void)
{
    agent_context_full_t *ctx = agent_context_get();

    /* Step landing action (motor control hold) */
    agent_action_landing_step();

    /* SAFETY: Real-time sync with IMU — check if hardware recovered */
    agent_sensor_sample_t sample;

    if (imu_get_sample(&sample) == AGENT_OK) {

        /* IMU OK → feed EKF + track recovery */
        if (imu_health() == AGENT_HEALTH_OK) {
            ekf_update(&sample);

            ctx->imu_consecutive_ok++;
            ctx->imu_consecutive_fail = 0U;

            /* SAFETY: LANDING recovery — IMU stably OK for N samples */
            if (ctx->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
                agent_confidence_set(1.0f);
                agent_recovery_register();

                agent_status_t st = agent_transition_to(AGENT_STATE_RECOVERY,
                                                        AGENT_REASON_NONE);
                if (st != AGENT_OK) {
                    (void)agent_transition_force(AGENT_STATE_RECOVERY,
                                                 (uint32_t)AGENT_REASON_NONE);
                }
                return;
            }
        } else {
            ctx->imu_consecutive_fail++;
            ctx->imu_consecutive_ok = 0U;
        }
    } else {
        ctx->imu_consecutive_fail++;
        ctx->imu_consecutive_ok = 0U;
    }

    /* SAFETY: Iteration-based fallback — if stuck in LANDING too long,
     * try recovery regardless. Counter stored in recovery_attempts. */
    ctx->recovery_attempts++;
    if (ctx->recovery_attempts >= 5000U) {   /* ~5 s @ 1 kHz */
        ctx->recovery_attempts = 0U;
        agent_confidence_set(0.5f);

        agent_status_t st = agent_transition_to(AGENT_STATE_RECOVERY,
                                                AGENT_REASON_TIMEOUT);
        if (st != AGENT_OK) {
            (void)agent_transition_force(AGENT_STATE_RECOVERY,
                                         (uint32_t)AGENT_REASON_TIMEOUT);
        }
        return;
    }

    /* Update control with last known EKF state */
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) {
        control_update(&state);
    }
}