#include "agent_states.h"
#include "agent_transitions.h"
#include "agent_decisions.h"
#include "agent_priority.h"
#include "../actions/agent_actions.h"
#include "../actions/agent_actions_ai.h"
#include "../safety/agent_safety.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_control_if.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../core/agent.h"

#include "../../sensor_layer/sal_core.h"
#include "../../sensor_layer/sal_types.h"

#include <zephyr/kernel.h>

static bool s_virtualize_retried = false;

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
    ctx->time_in_state_us = 0;
    agent_trace_state_enter(state);
    agent_log_event(AGENT_LOG_STATE_ENTER, (uint32_t)state, 0);
    switch (state) {
        case AGENT_STATE_MONITOR:
            (void)agent_action_set_mode(AGENT_MODE_NORMAL);
            agent_safety_deescalate();
            break;
        case AGENT_STATE_DEGRADED:
            (void)agent_action_set_mode(AGENT_MODE_DEGRADED);
            break;
        case AGENT_STATE_VIRTUALIZE:
            s_virtualize_retried = false;
            (void)agent_action_set_mode(AGENT_MODE_VIRTUAL);
            (void)agent_action_ai_trigger();
            break;
        case AGENT_STATE_FUSION:
            (void)agent_action_set_mode(AGENT_MODE_FUSION);
            break;
        case AGENT_STATE_RECOVERY:
            (void)agent_action_set_mode(AGENT_MODE_RECOVERY);
            break;
        case AGENT_STATE_EMERGENCY:
            (void)agent_action_set_mode(AGENT_MODE_EMERGENCY);
            (void)agent_action_emergency();
            break;
        case AGENT_STATE_LANDING:
            (void)agent_action_set_mode(AGENT_MODE_LANDING);
            break;
        default:
            break;
    }
}

void agent_state_exit(agent_state_t state)
{
    agent_log_event(AGENT_LOG_STATE_EXIT, (uint32_t)state, agent_time_in_state_us());
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

static bool sal_imu_get(agent_sensor_sample_t *sample)
{
    sal_imu_data_t sal_data;
    if (sal_get_imu(&sal_data) != SAL_OK) return false;
    if (sal_get_health("imu") != SAL_HEALTH_OK) return false;

    sample->ax = sal_data.ax;
    sample->ay = sal_data.ay;
    sample->az = sal_data.az;
    sample->gx = sal_data.gx;
    sample->gy = sal_data.gy;
    sample->gz = sal_data.gz;
    sample->timestamp_us = sal_data.timestamp_us;
    sample->health = AGENT_HEALTH_OK;
    return true;
}

void agent_state_boot(void)
{
    if (agent_safety_self_test() != AGENT_OK) {
        agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_NONE);
        return;
    }
    if (sal_get_health("imu") != SAL_HEALTH_OK) {
        agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_TIMEOUT);
        return;
    }
    if (ai_health() != AGENT_OK) {
        agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_CONFIDENCE);
        return;
    }
    if (ekf_health() != AGENT_OK) {
        agent_transition_to(AGENT_STATE_EMERGENCY, AGENT_REASON_INNOVATION);
        return;
    }
    agent_transition_to(AGENT_STATE_MONITOR, AGENT_REASON_NONE);
}

void agent_state_monitor(void)
{
    agent_sensor_sample_t sample;
    bool ok = sal_imu_get(&sample);

    if (ok) {
        agent_context_get()->imu_consecutive_ok++;
        agent_context_get()->imu_consecutive_fail = 0;
        (void)ekf_update(&sample);
    } else {
        agent_context_get()->imu_consecutive_ok = 0;
        agent_context_get()->imu_consecutive_fail++;
        if (agent_context_get()->imu_consecutive_fail >= AGENT_FAIL_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_TIMEOUT);
            return;
        }
    }

    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) (void)control_update(&state);
}

void agent_state_degraded(void)
{
    agent_sensor_sample_t sample;
    bool sample_ok = sal_imu_get(&sample);
    if (sample_ok && sal_get_health("imu") == SAL_HEALTH_OK) {
        agent_context_get()->imu_consecutive_ok++;
        if (agent_context_get()->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_RECOVERY, AGENT_REASON_NONE);
            return;
        }
    } else {
        agent_context_get()->imu_consecutive_ok = 0;
        agent_context_get()->imu_consecutive_fail++;
        if (agent_context_get()->imu_consecutive_fail >= AGENT_FAIL_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_INNOVATION);
            return;
        }
    }
    if (sample_ok) (void)ekf_update(&sample);
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) (void)control_update(&state);
}

void agent_state_virtualize(void)
{
    if (ai_output_ready()) {
        agent_sensor_sample_t virtual_sample;
        if (ai_get_output(&virtual_sample) == AGENT_OK) {
            (void)ekf_update(&virtual_sample);
            agent_transition_to(AGENT_STATE_FUSION, AGENT_REASON_CONFIDENCE);
            return;
        }
    }
    if (!s_virtualize_retried && ai_status() == AI_STATUS_ERROR) {
        s_virtualize_retried = true;
        (void)ai_trigger();
        return;
    }
    if (!s_virtualize_retried && ai_status() == AI_STATUS_IDLE) {
        s_virtualize_retried = true;
        (void)ai_trigger();
        return;
    }
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) (void)control_update(&state);
}

void agent_state_fusion(void)
{
    agent_sensor_sample_t real_sample, virtual_sample, fused;
    bool real_ok = sal_imu_get(&real_sample);
    bool virtual_ok = ai_output_ready();
    if (virtual_ok) (void)ai_get_output(&virtual_sample);

    if (real_ok && virtual_ok) {
        fused.ax = (real_sample.ax + virtual_sample.ax) / 2;
        fused.ay = (real_sample.ay + virtual_sample.ay) / 2;
        fused.az = (real_sample.az + virtual_sample.az) / 2;
        fused.gx = (real_sample.gx + virtual_sample.gx) / 2;
        fused.gy = (real_sample.gy + virtual_sample.gy) / 2;
        fused.gz = (real_sample.gz + virtual_sample.gz) / 2;
        fused.timestamp_us = agent_time_now_us();
        fused.health = AGENT_HEALTH_OK;
        (void)ekf_update(&fused);
    } else if (virtual_ok) {
        (void)ekf_update(&virtual_sample);
    } else if (real_ok && imu_health() == AGENT_HEALTH_OK) {
        (void)ekf_update(&real_sample);
        agent_context_get()->imu_consecutive_ok++;
        if (agent_context_get()->imu_consecutive_ok >= AGENT_RECOVER_CONFIRM_COUNT) {
            agent_transition_to(AGENT_STATE_RECOVERY, AGENT_REASON_NONE);
            return;
        }
    } else {
        agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_NONE);
        return;
    }
    agent_ekf_state_t state;
    if (ekf_get_state(&state) == AGENT_OK) (void)control_update(&state);
}

void agent_state_recovery(void)
{
    agent_sensor_sample_t sample;
    if (sal_imu_get(&sample) && imu_health() == AGENT_HEALTH_OK) {
        (void)ekf_update(&sample);
        agent_recovery_register();
        agent_transition_to(AGENT_STATE_MONITOR, AGENT_REASON_NONE);
        return;
    }
    agent_transition_to(AGENT_STATE_VIRTUALIZE, AGENT_REASON_DRIFT);
}

void agent_state_emergency(void)
{
    (void)agent_action_emergency_hold();
    if (agent_safety_can_land_standalone() ||
        agent_safety_can_land() ||
        agent_time_in_state_us() > AGENT_EMERGENCY_LAND_TIMEOUT_US) {
        agent_transition_to(AGENT_STATE_LANDING, AGENT_REASON_ENVELOPE);
        return;
    }
}

void agent_state_landing(void)
{
    (void)agent_action_landing_step();
    if (agent_safety_is_landed()) {
        /* Stay in LANDING until IMU recovers. */
    }
}