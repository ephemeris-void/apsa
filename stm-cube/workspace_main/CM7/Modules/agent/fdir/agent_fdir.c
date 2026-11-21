#include "agent_fdir.h"
#include "agent_fdir_detector.h"
#include "agent_fdir_isolator.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT FDIR — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 80 µs per update
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_fdir_status_t g_fdir;
static bool                           g_fdir_initialized = false;

/* ---------- Channel reset ---------- */
AGENT_INLINE_FAST void fdir_channel_reset(agent_fdir_channel_t *ch, agent_fdir_sensor_t s)
{
    memset(ch, 0, sizeof(*ch));   /* zero-init all fields including reserved[] */

    ch->sensor     = s;
    ch->state      = AGENT_FDIR_STATE_OK;
    ch->fault      = AGENT_FDIR_FAULT_NONE;
    ch->confidence = 1.0f;
    ch->isolated   = false;
}

/* ---------- INIT ---------- */
agent_status_t agent_fdir_init(void)
{
    if (g_fdir_initialized) {
        return AGENT_OK;
    }

    memset(&g_fdir, 0, sizeof(g_fdir));    /* zero-init including reserved[] */

    for (uint32_t i = 0U; i < AGENT_FDIR_MAX_SENSORS; i++) {
        fdir_channel_reset(&g_fdir.channels[i], (agent_fdir_sensor_t)i);
    }

    g_fdir.fault_total    = 0U;
    g_fdir.isolate_total  = 0U;
    g_fdir.recover_total  = 0U;
    g_fdir.last_update_us = agent_time_now_us();

    agent_fdir_detector_init();
    agent_fdir_isolator_init();

    g_fdir_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_fdir_reset(void)
{
    g_fdir_initialized = false;
    return agent_fdir_init();
}

/* ---------- UPDATE ---------- */
agent_status_t agent_fdir_update(void)
{
    if (!g_fdir_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_trace_safety_begin();

    for (uint32_t i = 0U; i < AGENT_FDIR_MAX_SENSORS; i++) {

        agent_fdir_channel_t *ch = &g_fdir.channels[i];

        if (ch->isolated) {
            continue;
        }

        agent_fdir_fault_t fault  = AGENT_FDIR_FAULT_NONE;
        uint32_t           reason = 0U;

        agent_status_t st = agent_fdir_detect(ch->sensor, &fault, &reason);

        if (st != AGENT_OK) {
            continue;
        }

        if (fault != AGENT_FDIR_FAULT_NONE) {
            ch->fault_count++;
            ch->ok_count = 0U;

            if (ch->fault_count == 1U) {
                ch->fault_first_us = agent_time_now_us();
            }

            ch->fault_last_us = agent_time_now_us();
            ch->fault         = fault;
            ch->confidence    = 0.5f;

            if (ch->fault_count >= AGENT_FAIL_CONFIRM_COUNT) {
                ch->state = AGENT_FDIR_STATE_FAILED;
                (void)agent_fdir_isolator_run(ch);
            } else {
                ch->state = AGENT_FDIR_STATE_DEGRADED;
            }

            agent_log_event(AGENT_LOG_IMU_FAIL,
                            ((uint32_t)ch->sensor << 16) | (uint32_t)fault,
                            reason);

        } else {
            ch->ok_count++;
            ch->fault_count = 0U;

            if (ch->ok_count >= AGENT_RECOVER_CONFIRM_COUNT) {

                if (ch->state != AGENT_FDIR_STATE_OK) {
                    g_fdir.recover_total++;
                    agent_log_event(AGENT_LOG_SENSOR_RECOVERED,
                                    (uint32_t)ch->sensor, 0U);
                }

                ch->state      = AGENT_FDIR_STATE_OK;
                ch->fault      = AGENT_FDIR_FAULT_NONE;
                ch->confidence = 1.0f;
            }
        }
    }

    g_fdir.last_update_us = agent_time_now_us();

    agent_trace_safety_end();
    return AGENT_OK;
}

/* ---------- QUERIES ---------- */
agent_fdir_state_t agent_fdir_get_state(agent_fdir_sensor_t s)
{
    if (s >= AGENT_FDIR_MAX_SENSORS) {
        return AGENT_FDIR_STATE_FAILED;
    }
    return g_fdir.channels[s].state;
}

agent_fdir_fault_t agent_fdir_get_fault(agent_fdir_sensor_t s)
{
    if (s >= AGENT_FDIR_MAX_SENSORS) {
        return AGENT_FDIR_FAULT_UNKNOWN;
    }
    return g_fdir.channels[s].fault;
}

float agent_fdir_get_confidence(agent_fdir_sensor_t s)
{
    if (s >= AGENT_FDIR_MAX_SENSORS) {
        return 0.0f;
    }
    return g_fdir.channels[s].confidence;
}

bool agent_fdir_is_isolated(agent_fdir_sensor_t s)
{
    if (s >= AGENT_FDIR_MAX_SENSORS) {
        return true;
    }
    return g_fdir.channels[s].isolated;
}

/* ---------- ISOLATE ---------- */
agent_status_t agent_fdir_isolate(agent_fdir_sensor_t s)
{
    if (s >= AGENT_FDIR_MAX_SENSORS) {
        return AGENT_ERR;
    }

    agent_fdir_channel_t *ch = &g_fdir.channels[s];

    if (ch->isolated) {
        return AGENT_OK;
    }

    ch->isolated     = true;
    ch->state        = AGENT_FDIR_STATE_ISOLATED;
    ch->isolation_us = agent_time_now_us();

    g_fdir.isolate_total++;

    agent_log_event(AGENT_LOG_SAFETY_ESCALATE,
                    (uint32_t)s, ch->isolation_us);

    return AGENT_OK;
}

/* ---------- RESTORE ---------- */
agent_status_t agent_fdir_restore(agent_fdir_sensor_t s)
{
    if (s >= AGENT_FDIR_MAX_SENSORS) {
        return AGENT_ERR;
    }

    agent_fdir_channel_t *ch = &g_fdir.channels[s];

    ch->isolated     = false;
    ch->state        = AGENT_FDIR_STATE_OK;
    ch->fault        = AGENT_FDIR_FAULT_NONE;
    ch->fault_count  = 0U;
    ch->ok_count     = 0U;
    ch->isolation_us = 0U;
    ch->confidence   = 1.0f;

    return AGENT_OK;
}

/* ---------- STATUS ---------- */
const agent_fdir_status_t *agent_fdir_status(void)
{
    return &g_fdir;
}