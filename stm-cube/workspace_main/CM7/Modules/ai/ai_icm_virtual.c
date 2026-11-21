#include "ai_icm_virtual.h"
#include "../agent/interfaces/agent_ai_if.h"
#include "../agent/core/agent_context.h"
#include "../agent/core/agent_config.h"
#include "../agent/trace/agent_log.h"

#include "stm32h7xx_hal.h"

#include "app_x-cube-ai.h"
#include "network.h"
#include "network_data.h"

#include <string.h>

/* ============================================================
 * AI VIRTUAL SENSOR WRAPPER — IMPLEMENTATION
 * ============================================================
 * Board:  NUCLEO-H755ZI-Q @ 480 MHz
 * Model:  virtual_sensor_v1_int8.tflite
 * WCET:   11 ms (measured, target 5 ms)
 * ============================================================ */

/* ---------- External AI Buffer Accessors (from app_x-cube-ai.c) ---------- */
extern void* AI_Get_Input_Buffer(void);
extern void* AI_Get_Output_Buffer(void);

/* ---------- State ---------- */
static bool         g_ai_initialized = false;
static ai_status_t  g_ai_status      = AI_STATUS_IDLE;
static uint32_t     g_ai_sample_idx  = 0;
static uint8_t      g_ai_input_cache[AI_SAMPLES * AI_CHANNELS];

/* ---------- Buffer Access ---------- */
void* ai_model_input_buffer(void)
{
    return AI_Get_Input_Buffer();
}

void* ai_model_output_buffer(void)
{
    return AI_Get_Output_Buffer();
}

/* ---------- INIT ---------- */
agent_status_t ai_model_init(void)
{
    /* X-CUBE-AI bootstrap */
    MX_X_CUBE_AI_Init();

    if (AI_Get_Input_Buffer() == NULL ||
        AI_Get_Output_Buffer() == NULL) {
        g_ai_status = AI_STATUS_ERROR;
        agent_log_event(AGENT_LOG_AI_FAIL, 1, 0);
        return AGENT_ERR_HW;
    }

    memset(g_ai_input_cache, 0, sizeof(g_ai_input_cache));

    g_ai_initialized = true;
    g_ai_status      = AI_STATUS_IDLE;
    g_ai_sample_idx  = 0;

    return AGENT_OK;
}

/* ---------- TRIGGER ---------- */
agent_status_t ai_model_trigger(void)
{
    if (!g_ai_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    if (g_ai_status == AI_STATUS_RUNNING) {
        return AGENT_OK;
    }

    /* Feed buffer to X-CUBE-AI */
    int8_t *ai_in = (int8_t *)AI_Get_Input_Buffer();
    memcpy(ai_in, g_ai_input_cache, sizeof(g_ai_input_cache));

    /* Clean D-Cache: push input buffer to memory */
    SCB_CleanDCache_by_Addr((uint32_t *)ai_in, AI_SAMPLES * 4);

    g_ai_status = AI_STATUS_RUNNING;

    uint32_t t_start = agent_time_now_us();

    /* Inference (blocking ~11 ms) */
    MX_X_CUBE_AI_Process();

    uint32_t dt = agent_time_now_us() - t_start;

    /* Invalidate D-Cache: read fresh output from memory */
    int8_t *ai_out = (int8_t *)AI_Get_Output_Buffer();
    SCB_InvalidateDCache_by_Addr((uint32_t *)ai_out, AI_SAMPLES * 4);

    /* SAFETY: Timeout — if inference takes > 100 ms, mark error */
    if (dt > 100000U) {
        g_ai_status = AI_STATUS_ERROR;
        agent_log_event(AGENT_LOG_AI_FAIL, 0xEE01U, dt);
        return AGENT_ERR;
    }

    g_ai_status = AI_STATUS_READY;

    return AGENT_OK;
}

/* ---------- POLL (non-blocking background inference) ---------- */
void ai_model_poll(void)
{
    if (!g_ai_initialized) {
        return;
    }

    /* Only trigger if IDLE — never block on RUNNING */
    if (g_ai_status == AI_STATUS_IDLE) {
        (void)ai_model_trigger();
    }
}

/* ---------- STATUS ---------- */
ai_status_t ai_model_status(void)
{
    return g_ai_status;
}

/* ---------- OUTPUT READY ---------- */
bool ai_model_output_ready(void)
{
    return (g_ai_status == AI_STATUS_READY);
}

/* ---------- GET OUTPUT SAMPLE ---------- */
agent_status_t ai_model_get_output(agent_sensor_sample_t *sample)
{
    if (sample == NULL) {
        return AGENT_ERR;
    }

    if (g_ai_status != AI_STATUS_READY) {
        return AGENT_ERR_NOT_INIT;
    }

    int8_t *ai_out = (int8_t *)AI_Get_Output_Buffer();

    /* Extract first sample from each channel, dequantize */
    int8_t o0 = ai_out[0 * AI_SAMPLES];
    int8_t o1 = ai_out[1 * AI_SAMPLES];
    int8_t o2 = ai_out[2 * AI_SAMPLES];
    int8_t o3 = ai_out[3 * AI_SAMPLES];

    float d0 = ((float)o0 - AI_INT8_OUT_ZP) * AI_INT8_OUT_SCALE;
    float d1 = ((float)o1 - AI_INT8_OUT_ZP) * AI_INT8_OUT_SCALE;
    float d2 = ((float)o2 - AI_INT8_OUT_ZP) * AI_INT8_OUT_SCALE;
    float d3 = ((float)o3 - AI_INT8_OUT_ZP) * AI_INT8_OUT_SCALE;

    /* Denormalize back to raw IMU units */
    sample->ax = (int16_t)(d0 * AI_NORM_STD_CH0 + AI_NORM_MEAN_CH0);
    sample->ay = (int16_t)(d1 * AI_NORM_STD_CH1 + AI_NORM_MEAN_CH1);
    sample->az = (int16_t)(d2 * AI_NORM_STD_CH2 + AI_NORM_MEAN_CH2);
    sample->gx = (int16_t)(d3 * AI_NORM_STD_CH3 + AI_NORM_MEAN_CH3);
    sample->gy = 0;
    sample->gz = 0;
    sample->timestamp_us = agent_time_now_us();
    sample->health       = AGENT_HEALTH_OK;

    g_ai_status = AI_STATUS_IDLE;

    return AGENT_OK;
}

/* ---------- HEALTH ---------- */
agent_status_t ai_model_health(void)
{
    if (g_ai_status == AI_STATUS_ERROR) {
        return AGENT_ERR_HW;
    }
    return AGENT_OK;
}

/* ---------- Feed Sensor to AI Input (called by agent actions) ---------- */
void ai_model_feed_sample(const agent_sensor_sample_t *s)
{
    if (!g_ai_initialized || s == NULL) {
        return;
    }

    /* Quantize → normalize → int8 */
    float n0 = ((float)s->ax - AI_NORM_MEAN_CH0) / AI_NORM_STD_CH0;
    float n1 = ((float)s->ay - AI_NORM_MEAN_CH1) / AI_NORM_STD_CH1;
    float n2 = ((float)s->az - AI_NORM_MEAN_CH2) / AI_NORM_STD_CH2;
    float n3 = ((float)s->gx - AI_NORM_MEAN_CH3) / AI_NORM_STD_CH3;

    int32_t q0 = (int32_t)(n0 / AI_INT8_IN_SCALE) + AI_INT8_IN_ZP;
    int32_t q1 = (int32_t)(n1 / AI_INT8_IN_SCALE) + AI_INT8_IN_ZP;
    int32_t q2 = (int32_t)(n2 / AI_INT8_IN_SCALE) + AI_INT8_IN_ZP;
    int32_t q3 = (int32_t)(n3 / AI_INT8_IN_SCALE) + AI_INT8_IN_ZP;

    if (q0 >  127) q0 =  127;  if (q0 < -128) q0 = -128;
    if (q1 >  127) q1 =  127;  if (q1 < -128) q1 = -128;
    if (q2 >  127) q2 =  127;  if (q2 < -128) q2 = -128;
    if (q3 >  127) q3 =  127;  if (q3 < -128) q3 = -128;

    g_ai_input_cache[0 * AI_SAMPLES + g_ai_sample_idx] = (uint8_t)q0;
    g_ai_input_cache[1 * AI_SAMPLES + g_ai_sample_idx] = (uint8_t)q1;
    g_ai_input_cache[2 * AI_SAMPLES + g_ai_sample_idx] = (uint8_t)q2;
    g_ai_input_cache[3 * AI_SAMPLES + g_ai_sample_idx] = (uint8_t)q3;

    g_ai_sample_idx++;
    if (g_ai_sample_idx >= AI_SAMPLES) {
        g_ai_sample_idx = 0;
    }
}

/* ============================================================
 * INTERFACE GLUE (agent_ai_if.h)
 * ============================================================ */

agent_status_t ai_init(void)
{
    return ai_model_init();
}

agent_status_t ai_trigger(void)
{
    return ai_model_trigger();
}

ai_status_t ai_status(void)
{
    return ai_model_status();
}

bool ai_output_ready(void)
{
    return ai_model_output_ready();
}

agent_status_t ai_get_output(agent_sensor_sample_t *sample)
{
    return ai_model_get_output(sample);
}

agent_status_t ai_health(void)
{
    return ai_model_health();
}

void ai_on_failure(void)
{
    g_ai_status = AI_STATUS_ERROR;
    agent_log_event(AGENT_LOG_AI_FAIL, 2, 0);
}