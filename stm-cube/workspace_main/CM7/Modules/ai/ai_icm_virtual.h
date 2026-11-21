#ifndef AI_ICM_VIRTUAL_H
#define AI_ICM_VIRTUAL_H

#include "../agent/core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AI VIRTUAL SENSOR WRAPPER
 * ============================================================
 * Board:  NUCLEO-H755ZI-Q @ 480 MHz
 * Model:  virtual_sensor_v1_int8.tflite (X-CUBE-AI)
 * Input:  [1, 4, 512, 1] int8 — 2048 bytes
 * Output: [1, 4, 512, 1] int8 — 2048 bytes
 * Latency: 11 ms (measured)
 * ============================================================ */

/* ---------- AI Model Interface ---------- */
agent_status_t  ai_model_init(void);
agent_status_t  ai_model_trigger(void);
void            ai_model_poll(void);
ai_status_t     ai_model_status(void);
bool            ai_model_output_ready(void);
agent_status_t  ai_model_get_output(agent_sensor_sample_t *sample);
agent_status_t  ai_model_health(void);

/* ---------- Buffer Access (from X-CUBE-AI) ---------- */
void*           ai_model_input_buffer(void);
void*           ai_model_output_buffer(void);

/* ---------- Feed Sensor Sample to AI Input Cache ---------- */
void            ai_model_feed_sample(const agent_sensor_sample_t *s);

/* ---------- int8 Quantization Params ---------- */
#define AI_INT8_IN_SCALE     (0.036893066f)
#define AI_INT8_IN_ZP        (-3)
#define AI_INT8_OUT_SCALE    (0.003224511f)
#define AI_INT8_OUT_ZP       (11)

#define AI_SAMPLES           512
#define AI_CHANNELS          4

/* ---------- Normalization Params ---------- */
#define AI_NORM_MEAN_CH0     (2032.53f)
#define AI_NORM_STD_CH0      (30.34f)
#define AI_NORM_MEAN_CH1     (2032.53f)
#define AI_NORM_STD_CH1      (30.34f)
#define AI_NORM_MEAN_CH2     (2032.53f)
#define AI_NORM_STD_CH2      (30.34f)
#define AI_NORM_MEAN_CH3     (2032.53f)
#define AI_NORM_STD_CH3      (30.34f)

#ifdef __cplusplus
}
#endif

#endif /* AI_ICM_VIRTUAL_H */