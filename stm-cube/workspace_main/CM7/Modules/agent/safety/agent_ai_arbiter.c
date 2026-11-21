#include "agent_ai_arbiter.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT AI ARBITER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per run
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_ai_arb_out_t g_last;
static bool                          g_arb_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_ai_arbiter_init(void)
{
    memset(&g_last, 0, sizeof(g_last));

    g_last.source           = AGENT_AI_ARB_SRC_NONE;
    g_last.result           = AGENT_AI_ARB_RESULT_FAIL;
    g_last.weight_real      = 0.0f;
    g_last.weight_ai        = 0.0f;
    g_last.confidence_milli = 0U;
    g_last.timestamp_us     = 0U;

    g_arb_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_ai_arbiter_reset(void)
{
    g_arb_initialized = false;
    return agent_ai_arbiter_init();
}

/* ---------- Weighted blend of int16 ---------- */
AGENT_INLINE_FAST int16_t blend_i16(int16_t a, float wa, int16_t b, float wb)
{
    float sum = wa + wb;

    if (sum < 1e-6f) {
        return a;
    }

    float v = ((float)a * wa + (float)b * wb) / sum;

    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;

    return (int16_t)v;
}

/* ---------- RUN ---------- */
agent_status_t agent_ai_arbiter_run(const agent_sensor_sample_t *real_sample,
                                    bool                          real_valid,
                                    const agent_sensor_sample_t *virtual_sample,
                                    bool                          virtual_valid,
                                    agent_ai_arb_out_t           *out)
{
    if (out == NULL)         return AGENT_ERR;
    if (!g_arb_initialized)  return AGENT_ERR_NOT_INIT;

    agent_ai_arb_out_t r;
    memset(&r, 0, sizeof(r));

    r.timestamp_us     = agent_time_now_us();
    r.confidence_milli = (uint32_t)(agent_confidence_get() * 1000.0f);

    if (real_valid && virtual_valid) {

        /* Both valid → weighted fusion */
        r.source      = AGENT_AI_ARB_SRC_FUSED;
        r.result      = AGENT_AI_ARB_RESULT_OK;
        r.weight_real = 0.5f;
        r.weight_ai   = 0.5f;

        r.output.ax = blend_i16(real_sample->ax, r.weight_real,
                                virtual_sample->ax, r.weight_ai);
        r.output.ay = blend_i16(real_sample->ay, r.weight_real,
                                virtual_sample->ay, r.weight_ai);
        r.output.az = blend_i16(real_sample->az, r.weight_real,
                                virtual_sample->az, r.weight_ai);
        r.output.gx = blend_i16(real_sample->gx, r.weight_real,
                                virtual_sample->gx, r.weight_ai);
        r.output.gy = blend_i16(real_sample->gy, r.weight_real,
                                virtual_sample->gy, r.weight_ai);
        r.output.gz = blend_i16(real_sample->gz, r.weight_real,
                                virtual_sample->gz, r.weight_ai);
        r.output.timestamp_us = r.timestamp_us;
        r.output.health       = AGENT_HEALTH_OK;

    } else if (real_valid) {

        /* Real only */
        r.source      = AGENT_AI_ARB_SRC_REAL_IMU;
        r.result      = AGENT_AI_ARB_RESULT_OK;
        r.weight_real = 1.0f;
        r.weight_ai   = 0.0f;
        r.output      = *real_sample;
        r.output.timestamp_us = r.timestamp_us;
        r.output.health       = AGENT_HEALTH_OK;

    } else if (virtual_valid) {

        /* AI fallback — flag degraded */
        r.source      = AGENT_AI_ARB_SRC_VIRTUAL_AI;
        r.result      = AGENT_AI_ARB_RESULT_FALLBACK;
        r.weight_real = 0.0f;
        r.weight_ai   = 1.0f;
        r.output      = *virtual_sample;
        r.output.timestamp_us = r.timestamp_us;
        r.output.health       = AGENT_HEALTH_DEGRADED;

        agent_log_event(AGENT_LOG_AI_SWITCH_INPUT, 0x01U, r.confidence_milli);

    } else {

        /* Both invalid → hold last EKF state */
        r.source      = AGENT_AI_ARB_SRC_HOLD;
        r.result      = AGENT_AI_ARB_RESULT_FAIL;
        r.weight_real = 0.0f;
        r.weight_ai   = 0.0f;

        agent_ekf_state_t st;
        if (ekf_get_state(&st) == AGENT_OK) {
            r.output.ax = 0;
            r.output.ay = 0;
            r.output.az = 0;
            r.output.gx = 0;
            r.output.gy = 0;
            r.output.gz = 0;
            r.output.timestamp_us = r.timestamp_us;
            r.output.health       = AGENT_HEALTH_FAIL;
        }

        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0x02U, 0U);
    }

    g_last = r;
    *out   = r;

    return (r.result == AGENT_AI_ARB_RESULT_FAIL) ? AGENT_ERR : AGENT_OK;
}

/* ---------- QUERIES ---------- */
agent_ai_arb_source_t agent_ai_arbiter_last_source(void)
{
    return g_last.source;
}

const agent_ai_arb_out_t *agent_ai_arbiter_last(void)
{
    return &g_last;
}