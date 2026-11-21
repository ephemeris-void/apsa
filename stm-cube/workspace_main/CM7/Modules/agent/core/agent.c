#include "main.h"
#include <stdio.h>
#include "agent.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_control_if.h"
#include "agent_context.h"
#include "agent_config.h"
#include "agent_main_hook.h"
#include "../states/agent_states.h"
#include "../states/agent_transitions.h"
#include "../health/agent_health.h"
#include "../health/agent_health_per_sensor.h"
#include "../decisions/agent_decisions.h"
#include "../decisions/agent_priority.h"
#include "../actions/agent_actions.h"
#include "../safety/agent_safety.h"
#include "../safety/agent_watchdog.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"

#include "ai_icm_virtual.h"   /* for ai_model_poll() */

/* ============================================================
 * AGENT — MAIN ENTRY
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Loop:  1 kHz (1000 µs period)
 * Deadline: 1000 µs hard
 * ============================================================ */

static bool g_agent_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_init(void)
{
    if (g_agent_initialized) {
        return AGENT_ERR;
    }

    /* Health FIRST — before any subsystem that reports health */
    agent_health_init();
    agent_health_per_sensor_init();

    agent_context_init();
    agent_trace_init();
    agent_watchdog_init();
    agent_safety_init();

    agent_state_set(AGENT_STATE_BOOT);
    agent_state_enter(AGENT_STATE_BOOT);

    agent_log_event(AGENT_LOG_INIT, 0U, 0U);

    g_agent_initialized = true;
    return AGENT_OK;
}

/* ---------- STEP (called @ 1 kHz) ---------- */
agent_status_t agent_step(void)
{
    if (!g_agent_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    /* 0. Health poll — ONCE per loop */
    agent_health_poll_all();

    /* 0.5 AI poll — background inference (non-blocking) */
    ai_model_poll();

    /* DEBUG — health state */
    {
        extern UART_HandleTypeDef huart3;
        char _b[128];
        int _n = sprintf(_b, "H:IMU=%d AI=%d EKF=%d CTRL=%d SYS=%d OK=%lu FL=%lu\r\n",
            (int)agent_health_get(AGENT_HEALTH_SENSOR_IMU),
            (int)agent_health_get(AGENT_HEALTH_SENSOR_AI),
            (int)agent_health_get(AGENT_HEALTH_SENSOR_EKF),
            (int)agent_health_get(AGENT_HEALTH_SENSOR_CONTROL),
            (int)agent_health_system(),
            (unsigned long)agent_context_get()->imu_consecutive_ok,
            (unsigned long)agent_context_get()->imu_consecutive_fail);
        HAL_UART_Transmit(&huart3, (uint8_t*)_b, _n, 50);
    }

    agent_iteration_begin();

    /* 1. Watchdog kick */
    agent_watchdog_kick();

    /* 2. Safety check */
    if (!agent_safety_check()) {
        agent_safety_escalate();
        agent_iteration_end();
        return AGENT_ERR;
    }

    /* 3. Decision phase */
    agent_decision_t decision;

    if (agent_decide(&decision) != AGENT_OK) {
        agent_fault_register(AGENT_FAULT_DEADLINE_MISS);
        agent_iteration_end();
        return AGENT_ERR;
    }

    /* 4. Transition check */
    if (decision.next_state != agent_get_state()) {
        agent_transition_to(decision.next_state, decision.reason_code);
    }

    /* 5. Action execution */
    if (agent_action_execute(&decision) != AGENT_OK) {
        agent_fault_register(AGENT_FAULT_DEADLINE_MISS);
        agent_iteration_end();
        return AGENT_ERR;
    }

    /* 6. State handler */
    agent_state_run(agent_get_state());

    /* 7. Deadline check */
    if (!agent_deadline_check()) {
        agent_fault_register(AGENT_FAULT_DEADLINE_MISS);
        agent_log_event(AGENT_LOG_DEADLINE_MISS, 0U, 0U);
    }

    agent_iteration_end();

    return AGENT_OK;
}

/* ---------- SHUTDOWN ---------- */
agent_status_t agent_shutdown(void)
{
    if (!g_agent_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_state_enter(AGENT_STATE_LANDING);
    agent_log_event(AGENT_LOG_SHUTDOWN, 0U, 0U);

    g_agent_initialized = false;
    return AGENT_OK;
}

/* ---------- STATE ACCESS ---------- */
agent_state_t agent_get_state(void)
{
    return agent_context_get()->current_state;
}

const char* agent_get_state_name(agent_state_t s)
{
    static const char * const names[AGENT_STATE_MAX] = {
        "BOOT", "MONITOR", "DEGRADED", "VIRTUALIZE",
        "FUSION", "RECOVERY", "EMERGENCY", "LANDING"
    };

    if (s >= AGENT_STATE_MAX) {
        return "UNKNOWN";
    }
    return names[s];
}

agent_health_t agent_get_health(void)
{
    return agent_context_get()->system_health;
}

float agent_get_confidence(void)
{
    return agent_context_get()->confidence;
}

/* ---------- DIAGNOSTICS ---------- */
uint32_t agent_get_iteration_count(void)
{
    return agent_context_get()->iteration_count;
}

uint32_t agent_get_fault_count(void)
{
    return agent_context_get()->fault_count;
}

uint32_t agent_get_recovery_count(void)
{
    return agent_context_get()->recovery_count;
}

uint32_t agent_get_wcet_max_us(void)
{
    return agent_context_get()->loop_wcet_max_us;
}

uint32_t agent_get_deadline_miss_count(void)
{
    return agent_context_get()->deadline_miss_count;
}

/* ---------- EXTERNAL EVENT INJECTION ----------
 * PERMANENT FIX: Calls driver-internal on_failure() so
 * icm_read_sample() returns error until recovery, preventing
 * the FUSION↔RECOVERY↔VIRTUALIZE cycle.
 * ============================================================ */
void agent_event_imu_fail(void)
{
    /* Set driver-internal health = FAIL so recovery logic triggers */
    imu_on_failure();
    /* Sync context health */
    agent_health_set_imu(AGENT_HEALTH_FAIL);
    agent_health_set_ai(AGENT_HEALTH_OK);
    agent_fault_register(AGENT_FAULT_IMU_FAIL);
    agent_log_event(AGENT_LOG_IMU_FAIL, 0U, 0U);
}

void agent_event_ai_fail(void)
{
    /* Set driver-internal health = FAIL */
    ai_on_failure();
    agent_health_set_ai(AGENT_HEALTH_FAIL);
    agent_fault_register(AGENT_FAULT_AI_FAIL);
    agent_log_event(AGENT_LOG_AI_FAIL, 0U, 0U);
}

void agent_event_ekf_diverge(void)
{
    agent_fault_register(AGENT_FAULT_EKF_DIVERGE);
    agent_log_event(AGENT_LOG_EKF_DIVERGE, 0U, 0U);
}

void agent_event_ipc_timeout(void)
{
    agent_fault_register(AGENT_FAULT_IPC_TIMEOUT);
    agent_log_event(AGENT_LOG_IPC_TIMEOUT, 0U, 0U);
}

void agent_event_watchdog(void)
{
    agent_fault_register(AGENT_FAULT_WATCHDOG);
    agent_log_event(AGENT_LOG_WATCHDOG, 0U, 0U);
}

void agent_event_sensor_recovered(void)
{
    agent_health_set_imu(AGENT_HEALTH_OK);
    agent_log_event(AGENT_LOG_SENSOR_RECOVERED, 0U, 0U);
}