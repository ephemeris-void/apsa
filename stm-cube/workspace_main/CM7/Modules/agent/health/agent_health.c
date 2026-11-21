#include "agent_health.h"
#include "agent_health_per_sensor.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT HEALTH — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per call
 * ============================================================ */

static AGENT_DTCM agent_health_state_t g_health;
static bool                            g_health_initialized = false;

/* ---------- Combine (max severity) ---------- */
AGENT_INLINE_FAST agent_health_t combine_health(agent_health_t a, agent_health_t b)
{
    return (a > b) ? a : b;
}

/* ---------- INIT ---------- */
agent_status_t agent_health_init(void)
{
    memset(&g_health, 0, sizeof(g_health));

    for (uint32_t i = 0U; i < AGENT_HEALTH_SENSOR_COUNT; i++) {
        agent_health_channel_t *ch = &g_health.channels[i];
        ch->sensor          = (agent_health_sensor_t)i;
        ch->health          = AGENT_HEALTH_UNKNOWN;
        ch->previous        = AGENT_HEALTH_UNKNOWN;
        ch->fail_count      = 0U;
        ch->ok_count        = 0U;
        ch->last_update_us  = 0U;
        ch->last_fail_us    = 0U;
        ch->last_recover_us = 0U;
        ch->total_fails     = 0U;
        ch->total_recovers  = 0U;
    }

    g_health.system         = AGENT_HEALTH_UNKNOWN;
    g_health.last_update_us = agent_time_now_us();
    g_health.worst_sensor   = 0U;

    agent_health_per_sensor_init();

    g_health_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_health_reset(void)
{
    g_health_initialized = false;
    return agent_health_init();
}

/* ---------- SET ---------- */
agent_status_t agent_health_set(agent_health_sensor_t sensor, agent_health_t health)
{
    if (!g_health_initialized)               return AGENT_ERR_NOT_INIT;
    if (sensor >= AGENT_HEALTH_SENSOR_COUNT) return AGENT_ERR;

    agent_health_channel_t *ch = &g_health.channels[sensor];
    uint32_t now = agent_time_now_us();

    ch->previous       = ch->health;
    ch->health         = health;
    ch->last_update_us = now;

    /* ---------- IMU-specific counter update ---------- */
    if (sensor == AGENT_HEALTH_SENSOR_IMU) {

        agent_context_full_t *ctx = agent_context_get();

        if (health == AGENT_HEALTH_FAIL) {
            ch->fail_count++;
            ch->ok_count     = 0U;
            ch->last_fail_us = now;
            ch->total_fails++;

            ctx->imu_consecutive_fail++;
            ctx->imu_consecutive_ok = 0U;

        } else if (health == AGENT_HEALTH_OK) {
            ch->ok_count++;
            ch->fail_count = 0U;

            ctx->imu_consecutive_ok++;
            ctx->imu_consecutive_fail = 0U;

            if (ch->previous == AGENT_HEALTH_FAIL &&
                ch->ok_count >= AGENT_HEALTH_RECOVER_CONFIRM) {

                ch->last_recover_us = now;
                ch->total_recovers++;
                agent_log_event(AGENT_LOG_SENSOR_RECOVERED,
                                (uint32_t)sensor, ch->ok_count);
            }

        } else if (health == AGENT_HEALTH_DEGRADED) {
            ch->ok_count++;
            ch->fail_count = 0U;

            ctx->imu_consecutive_ok++;
            ctx->imu_consecutive_fail = 0U;
        }

    } else {
        /* Non-IMU sensors */
        if (health == AGENT_HEALTH_FAIL) {
            ch->fail_count++;
            ch->ok_count     = 0U;
            ch->last_fail_us = now;
            ch->total_fails++;

            if (ch->fail_count == AGENT_HEALTH_FAIL_CONFIRM) {
                agent_log_event(AGENT_LOG_IMU_FAIL,
                                ((uint32_t)sensor << 16) | 0x01U,
                                ch->fail_count);
            }
        } else if (health == AGENT_HEALTH_OK) {
            ch->ok_count++;
            ch->fail_count = 0U;

            if (ch->previous == AGENT_HEALTH_FAIL &&
                ch->ok_count >= AGENT_HEALTH_RECOVER_CONFIRM) {

                ch->last_recover_us = now;
                ch->total_recovers++;
                agent_log_event(AGENT_LOG_SENSOR_RECOVERED,
                                (uint32_t)sensor, ch->ok_count);
            }
        } else if (health == AGENT_HEALTH_DEGRADED) {
            ch->ok_count = 0U;
        }
    }

    return AGENT_OK;
}

/* ---------- UPDATE (aggregate) ----------
 * SAFETY: System health determined by IMU + AI + EKF only.
 *         Control sensor contributes to confidence but does
 *         not force system into FAIL.
 *
 *         EKF fail → system DEGRADED (not FAIL) to allow
 *         auto-recovery without triggering EMERGENCY.
 */
agent_status_t agent_health_update(void)
{
    if (!g_health_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_health_t imu_h = g_health.channels[AGENT_HEALTH_SENSOR_IMU].health;
    agent_health_t ai_h  = g_health.channels[AGENT_HEALTH_SENSOR_AI].health;
    agent_health_t ekf_h = g_health.channels[AGENT_HEALTH_SENSOR_EKF].health;

    agent_health_t system = AGENT_HEALTH_OK;

    /* IMU is critical */
    if (imu_h == AGENT_HEALTH_FAIL) {
        system = AGENT_HEALTH_FAIL;
    } else if (imu_h == AGENT_HEALTH_DEGRADED) {
        system = AGENT_HEALTH_DEGRADED;
    }

    /* AI is critical */
    if (ai_h == AGENT_HEALTH_FAIL) {
        system = AGENT_HEALTH_FAIL;
    } else if (ai_h == AGENT_HEALTH_DEGRADED &&
               system != AGENT_HEALTH_FAIL) {
        system = AGENT_HEALTH_DEGRADED;
    }

    /* EKF fail → DEGRADED only (auto-recovery active) */
    if (ekf_h == AGENT_HEALTH_FAIL) {
        if (system == AGENT_HEALTH_OK) {
            system = AGENT_HEALTH_DEGRADED;
        }
    }

    g_health.system         = system;
    g_health.last_update_us = agent_time_now_us();

    /* Sync to context */
    agent_context_full_t *ctx = agent_context_get();
    ctx->system_health = system;

    return AGENT_OK;
}

/* ---------- GET ---------- */
agent_health_t agent_health_get(agent_health_sensor_t sensor)
{
    if (sensor >= AGENT_HEALTH_SENSOR_COUNT) {
        return AGENT_HEALTH_FAIL;
    }
    return g_health.channels[sensor].health;
}

agent_health_t agent_health_system(void)
{
    return g_health.system;
}

const agent_health_channel_t *agent_health_channel(agent_health_sensor_t sensor)
{
    if (sensor >= AGENT_HEALTH_SENSOR_COUNT) {
        return NULL;
    }
    return &g_health.channels[sensor];
}

const agent_health_state_t *agent_health_state(void)
{
    return &g_health;
}