#include "agent_ai_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../safety/agent_safety.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT AI SAFETY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 30 µs per validate
 * ============================================================ */

#define AI_SAFETY_MAX_AGE_US         20000U
#define AI_SAFETY_ENVELOPE_MARGIN    0.90f
#define AI_SAFETY_ACCEL_LIMIT_G      16.0f
#define AI_SAFETY_GYRO_LIMIT_DPS     2000.0f
#define AI_SAFETY_MG_TO_LSB          2048.0f
#define AI_SAFETY_DPS_TO_LSB         16.4f

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_ai_safety_report_t g_ai_safety_last;
static uint32_t                            g_ai_safety_last_update_us = 0U;
static bool                                g_ai_safety_initialized    = false;

/* ---------- abs_f (inline) ---------- */
AGENT_INLINE_FAST float abs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ---------- INIT ---------- */
agent_status_t agent_ai_safety_init(void)
{
    g_ai_safety_last.result          = AGENT_AI_SAFETY_OK;
    g_ai_safety_last.reason_code     = 0U;
    g_ai_safety_last.timestamp_us    = 0U;
    g_ai_safety_last.wcet_us         = 0U;
    g_ai_safety_last.confidence      = 0.0f;
    g_ai_safety_last.envelope_margin = 1.0f;
    g_ai_safety_last.accepted        = false;

    g_ai_safety_last_update_us = 0U;
    g_ai_safety_initialized    = true;

    return AGENT_OK;
}

agent_status_t agent_ai_safety_reset(void)
{
    g_ai_safety_initialized = false;
    return agent_ai_safety_init();
}

/* ---------- ENVELOPE CHECK ---------- */
agent_status_t agent_ai_safety_check_envelope(const agent_sensor_sample_t *sample,
                                              float                       *margin_out)
{
    if (sample == NULL || margin_out == NULL) {
        return AGENT_ERR;
    }

    float ax = (float)sample->ax / AI_SAFETY_MG_TO_LSB;
    float ay = (float)sample->ay / AI_SAFETY_MG_TO_LSB;
    float az = (float)sample->az / AI_SAFETY_MG_TO_LSB;

    float gx = (float)sample->gx / AI_SAFETY_DPS_TO_LSB;
    float gy = (float)sample->gy / AI_SAFETY_DPS_TO_LSB;
    float gz = (float)sample->gz / AI_SAFETY_DPS_TO_LSB;

    /* Accel envelope */
    if (abs_f(ax) > AI_SAFETY_ACCEL_LIMIT_G ||
        abs_f(ay) > AI_SAFETY_ACCEL_LIMIT_G ||
        abs_f(az) > AI_SAFETY_ACCEL_LIMIT_G) {
        *margin_out = 0.0f;
        return AGENT_ERR;
    }

    /* Gyro envelope */
    if (abs_f(gx) > AI_SAFETY_GYRO_LIMIT_DPS ||
        abs_f(gy) > AI_SAFETY_GYRO_LIMIT_DPS ||
        abs_f(gz) > AI_SAFETY_GYRO_LIMIT_DPS) {
        *margin_out = 0.0f;
        return AGENT_ERR;
    }

    /* Worst-case margin */
    float worst = 0.0f;
    if (abs_f(ax) > worst) worst = abs_f(ax);
    if (abs_f(ay) > worst) worst = abs_f(ay);
    if (abs_f(az) > worst) worst = abs_f(az);

    *margin_out = 1.0f - (worst / AI_SAFETY_ACCEL_LIMIT_G);

    if (*margin_out < AI_SAFETY_ENVELOPE_MARGIN) {
        return AGENT_ERR;
    }

    return AGENT_OK;
}

/* ---------- FRESHNESS CHECK ---------- */
agent_status_t agent_ai_safety_check_freshness(uint32_t last_update_us,
                                               uint32_t max_age_us)
{
    uint32_t now = agent_time_now_us();
    uint32_t age = now - last_update_us;

    if (age > max_age_us) {
        return AGENT_ERR_TIMEOUT;
    }
    return AGENT_OK;
}

/* ---------- CONFIDENCE CHECK ---------- */
agent_status_t agent_ai_safety_check_confidence(float confidence, float min_confidence)
{
    if (confidence < min_confidence) {
        return AGENT_ERR;
    }
    return AGENT_OK;
}

/* ---------- MAIN VALIDATE ---------- */
agent_status_t agent_ai_safety_validate(const agent_sensor_sample_t *sample,
                                        agent_ai_safety_report_t    *report_out)
{
    if (sample == NULL || report_out == NULL) {
        return AGENT_ERR;
    }

    if (!g_ai_safety_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    uint32_t t0 = agent_time_now_us();

    agent_ai_safety_report_t rep;
    rep.result          = AGENT_AI_SAFETY_REJECTED;
    rep.reason_code     = 0U;
    rep.timestamp_us    = t0;
    rep.wcet_us         = 0U;
    rep.confidence      = agent_confidence_get();
    rep.envelope_margin = 0.0f;
    rep.accepted        = false;

    /* Gate 1: AI health */
    if (ai_health() != AGENT_OK) {
        rep.result      = AGENT_AI_SAFETY_REJECTED;
        rep.reason_code = 0x01U;
        goto done;
    }

    /* Gate 2: AI status */
    if (ai_status() == AI_STATUS_ERROR) {
        rep.result      = AGENT_AI_SAFETY_REJECTED;
        rep.reason_code = 0x02U;
        goto done;
    }

    /* Gate 3: Freshness */
    if (agent_ai_safety_check_freshness(g_ai_safety_last_update_us,
                                        AI_SAFETY_MAX_AGE_US) != AGENT_OK) {
        rep.result      = AGENT_AI_SAFETY_STALE;
        rep.reason_code = 0x03U;
        goto done;
    }

    /* Gate 4: EKF divergence */
    if (ekf_has_diverged()) {
        rep.result      = AGENT_AI_SAFETY_DIVERGED;
        rep.reason_code = 0x04U;
        goto done;
    }

    /* Gate 5: Envelope */
    float margin = 0.0f;
    if (agent_ai_safety_check_envelope(sample, &margin) != AGENT_OK) {
        rep.result          = AGENT_AI_SAFETY_ENVELOPE_FAIL;
        rep.reason_code     = 0x05U;
        rep.envelope_margin = margin;
        goto done;
    }
    rep.envelope_margin = margin;

    /* Gate 6: Confidence */
    if (agent_ai_safety_check_confidence(rep.confidence,
                                         AGENT_AI_CONFIDENCE_THRESHOLD) != AGENT_OK) {
        rep.result      = AGENT_AI_SAFETY_DEGRADED;
        rep.reason_code = 0x06U;
        goto done;
    }

    /* All gates passed */
    rep.result      = AGENT_AI_SAFETY_OK;
    rep.reason_code = 0U;
    rep.accepted    = true;

done:
    g_ai_safety_last_update_us = t0;

    rep.wcet_us = agent_time_now_us() - t0;

    g_ai_safety_last = rep;
    *report_out      = rep;

    if (!rep.accepted) {
        agent_log_event(AGENT_LOG_ENVELOPE_VIOLATE,
                        rep.reason_code,
                        (uint32_t)(rep.confidence * 1000.0f));
    }

    return rep.accepted ? AGENT_OK : AGENT_ERR;
}

/* ---------- HEALTH ---------- */
bool agent_ai_safety_is_healthy(void)
{
    return (g_ai_safety_last.result == AGENT_AI_SAFETY_OK);
}

/* ---------- LAST ---------- */
const agent_ai_safety_report_t *agent_ai_safety_last(void)
{
    return &g_ai_safety_last;
}