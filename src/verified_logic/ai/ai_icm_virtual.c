#include "ai_icm_virtual.h"
#include "../agent/interfaces/agent_ai_if.h"
#include "../agent/core/agent_context.h"
#include "../agent/core/agent_config.h"
#include "../agent/trace/agent_log.h"

#include "hal/hal_cache.h"
#include "hal/hal_time.h"
#include "common/logger.h"

#include "../X-CUBE-AI/App/ai_wrapper.h"
#include "network.h"
#include "network_data.h"

#include "verified_logic/sensor_layer/sal_core.h"
#include "verified_logic/sensor_layer/sal_types.h"

#include <string.h>
#include <zephyr/kernel.h>

static int8_t g_ai_in_buffer[AI_NETWORK_IN_1_SIZE_BYTES];
static int8_t g_ai_out_buffer[AI_NETWORK_OUT_1_SIZE_BYTES];

static bool         g_ai_initialized = false;
static ai_status_t  g_ai_status      = AI_STATUS_IDLE;
static uint32_t     g_ai_sample_idx  = 0;
static uint8_t      g_ai_input_cache[AI_SAMPLES * AI_CHANNELS];

void* ai_model_input_buffer(void)  { return g_ai_in_buffer; }
void* ai_model_output_buffer(void) { return g_ai_out_buffer; }

static const float g_mm_min[4] = { AI_MM_MIN_CH0, AI_MM_MIN_CH1,
                                   AI_MM_MIN_CH2, AI_MM_MIN_CH3 };
static const float g_mm_max[4] = { AI_MM_MAX_CH0, AI_MM_MAX_CH1,
                                   AI_MM_MAX_CH2, AI_MM_MAX_CH3 };

agent_status_t ai_model_init(void)
{
    if (g_ai_initialized) return AGENT_OK;

    if (ai_wrapper_init() != 0) {
        g_ai_status = AI_STATUS_ERROR;
        LOG_E(LOG_TAG_AI, "[AI] wrapper init FAILED");
        return AGENT_ERR_HW;
    }

    memset(g_ai_input_cache, 0, sizeof(g_ai_input_cache));

    g_ai_initialized = true;
    g_ai_status      = AI_STATUS_IDLE;
    g_ai_sample_idx  = 0;

    LOG_I(LOG_TAG_AI, "[AI] init OK");
    return AGENT_OK;
}

agent_status_t ai_model_trigger(void)
{
    if (!g_ai_initialized) return AGENT_ERR_NOT_INIT;
    if (g_ai_status == AI_STATUS_RUNNING) return AGENT_OK;

    memcpy(g_ai_in_buffer, g_ai_input_cache, sizeof(g_ai_input_cache));

    g_ai_status = AI_STATUS_RUNNING;
    int rc = ai_wrapper_run(g_ai_in_buffer, g_ai_out_buffer);
    if (rc != 0) {
        g_ai_status = AI_STATUS_ERROR;
        return AGENT_ERR_HW;
    }
    g_ai_status = AI_STATUS_READY;
    return AGENT_OK;
}

ai_status_t ai_model_status(void)  { return g_ai_status; }
bool ai_model_output_ready(void)   { return (g_ai_status == AI_STATUS_READY); }

static float unscale_ch(int ch, int8_t q_out)
{
    float d = ((float)q_out - (float)AI_INT8_OUT_ZP) * AI_INT8_OUT_SCALE;
    if (d < -1.0f) d = -1.0f;
    if (d >  1.0f) d =  1.0f;
    return (d + 1.0f) * 0.5f * (g_mm_max[ch] - g_mm_min[ch]) + g_mm_min[ch];
}

agent_status_t ai_model_get_output(agent_sensor_sample_t *sample)
{
    if (sample == NULL) return AGENT_ERR;
    if (g_ai_status != AI_STATUS_READY) return AGENT_ERR_NOT_INIT;

    int8_t o0 = g_ai_out_buffer[0 * AI_CHANNELS + 0];
    int8_t o1 = g_ai_out_buffer[0 * AI_CHANNELS + 1];
    int8_t o2 = g_ai_out_buffer[0 * AI_CHANNELS + 2];
    int8_t o3 = g_ai_out_buffer[0 * AI_CHANNELS + 3];

    float r0 = unscale_ch(0, o0);
    float r1 = unscale_ch(1, o1);
    float r2 = unscale_ch(2, o2);
    float r3 = unscale_ch(3, o3);

    if (r0 >  32767.0f) r0 =  32767.0f;
    if (r0 < -32768.0f) r0 = -32768.0f;
    if (r1 >  32767.0f) r1 =  32767.0f;
    if (r1 < -32768.0f) r1 = -32768.0f;
    if (r2 >  32767.0f) r2 =  32767.0f;
    if (r2 < -32768.0f) r2 = -32768.0f;
    if (r3 >   3276.7f) r3 =   3276.7f;
    if (r3 <  -3276.8f) r3 =  -3276.8f;

    sample->ax = (int16_t)r0;
    sample->ay = (int16_t)r1;
    sample->az = (int16_t)r2;
    sample->gx = (int16_t)(r3 * 10.0f);
    sample->gy = 0;
    sample->gz = 0;
    sample->timestamp_us = agent_time_now_us();
    sample->health       = AGENT_HEALTH_OK;

    /* Do NOT reset status — the caller transitions to FUSION on the
     * same iteration. Status is reset by the next ai_model_trigger(). */
    return AGENT_OK;
}

agent_status_t ai_model_health(void)
{
    return (g_ai_status == AI_STATUS_ERROR) ? AGENT_ERR_HW : AGENT_OK;
}

void ai_model_feed_sample(const agent_sensor_sample_t *s)
{
    if (!g_ai_initialized || s == NULL) return;

    sal_mag_data_t mag;
    float mag_x = 0.0f;
    if (sal_get_mag(&mag) == SAL_OK && mag.health == SAL_HEALTH_OK) {
        mag_x = mag.mag_ut[0];
    }

    const float raw[4] = {
        (float)s->ax, (float)s->ay, (float)s->az, mag_x
    };

    for (int i = 0; i < 4; i++) {
        float norm = 2.0f * ((raw[i] - g_mm_min[i]) /
                             (g_mm_max[i] - g_mm_min[i])) - 1.0f;
        int32_t q = (int32_t)(norm / AI_INT8_IN_SCALE) + AI_INT8_IN_ZP;
        if (q >  127) q =  127;
        if (q < -128) q = -128;

        g_ai_input_cache[g_ai_sample_idx * AI_CHANNELS + i] = (uint8_t)q;
    }

    g_ai_sample_idx++;
    if (g_ai_sample_idx >= AI_SAMPLES) {
        g_ai_sample_idx = 0;
    }
}

agent_status_t ai_init(void)              { return ai_model_init(); }
agent_status_t ai_trigger(void)           { return ai_model_trigger(); }
ai_status_t    ai_status(void)            { return ai_model_status(); }
bool           ai_output_ready(void)      { return ai_model_output_ready(); }
agent_status_t ai_get_output(agent_sensor_sample_t *s) { return ai_model_get_output(s); }
agent_status_t ai_health(void)            { return ai_model_health(); }

void ai_on_failure(void)
{
    g_ai_status = AI_STATUS_ERROR;
    agent_log_event(AGENT_LOG_AI_FAIL, 2U, 0U);
}