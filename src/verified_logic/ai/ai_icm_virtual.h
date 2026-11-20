#ifndef AI_ICM_VIRTUAL_H
#define AI_ICM_VIRTUAL_H

#include "../agent/core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

agent_status_t  ai_model_init(void);
agent_status_t  ai_model_trigger(void);
void            ai_model_poll(void);
ai_status_t     ai_model_status(void);
bool            ai_model_output_ready(void);
agent_status_t  ai_model_get_output(agent_sensor_sample_t *sample);
agent_status_t  ai_model_health(void);

void*           ai_model_input_buffer(void);
void*           ai_model_output_buffer(void);

void            ai_model_feed_sample(const agent_sensor_sample_t *s);

/* ---------- int8 Quantization (from TFLite) ---------- */
#define AI_INT8_IN_SCALE     (0.0078431377f)
#define AI_INT8_IN_ZP        (-1)
#define AI_INT8_OUT_SCALE    (0.0047876266f)
#define AI_INT8_OUT_ZP       (71)

#define AI_SAMPLES           512
#define AI_CHANNELS          4

/* ---------- MinMaxScaler Ranges (from Colab) ---------- */
#define AI_MM_MIN_CH0    (-20359.0f)   /* ax */
#define AI_MM_MAX_CH0    ( 23307.0f)
#define AI_MM_MIN_CH1    (-19342.0f)   /* ay */
#define AI_MM_MAX_CH1    ( 16572.0f)
#define AI_MM_MIN_CH2    ( -4823.0f)   /* az */
#define AI_MM_MAX_CH2    ( 14203.0f)
#define AI_MM_MIN_CH3    (    -6.8f)   /* mag_x */
#define AI_MM_MAX_CH3    (   197.9f)

#ifdef __cplusplus
}
#endif

#endif /* AI_ICM_VIRTUAL_H */