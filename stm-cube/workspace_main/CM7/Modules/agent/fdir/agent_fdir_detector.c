#include "agent_fdir_detector.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT FDIR DETECTOR — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 60 µs per detect
 * ============================================================ */

#define FDIR_STUCK_THRESHOLD   5U
#define FDIR_SPIKE_DELTA       10000
#define FDIR_RANGE_MIN         (-32000)
#define FDIR_RANGE_MAX         (32000)

/* ---------- Per-sensor state ---------- */
typedef struct {
    int16_t  last_ax;
    int16_t  last_ay;
    int16_t  last_az;
    int16_t  last_gx;
    int16_t  last_gy;
    int16_t  last_gz;
    uint32_t stuck_count;
    uint32_t last_update_us;
    bool     initialized;
} fdir_imu_ctx_t;

typedef struct {
    uint32_t last_update_us;
    float    last_confidence;
    bool     initialized;
} fdir_ai_ctx_t;

typedef struct {
    float    last_pos[3];
    float    last_vel[3];
    uint32_t last_update_us;
    bool     initialized;
} fdir_ekf_ctx_t;

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM fdir_imu_ctx_t g_imu_ctx;
static AGENT_DTCM fdir_ai_ctx_t  g_ai_ctx;
static AGENT_DTCM fdir_ekf_ctx_t g_ekf_ctx;

/* ---------- INIT ---------- */
agent_status_t agent_fdir_detector_init(void)
{
    g_imu_ctx.last_ax        = 0;
    g_imu_ctx.last_ay        = 0;
    g_imu_ctx.last_az        = 0;
    g_imu_ctx.last_gx        = 0;
    g_imu_ctx.last_gy        = 0;
    g_imu_ctx.last_gz        = 0;
    g_imu_ctx.stuck_count    = 0U;
    g_imu_ctx.last_update_us = 0U;
    g_imu_ctx.initialized    = true;

    g_ai_ctx.last_update_us   = 0U;
    g_ai_ctx.last_confidence  = 0.0f;
    g_ai_ctx.initialized      = true;

    g_ekf_ctx.last_pos[0]     = 0.0f;
    g_ekf_ctx.last_pos[1]     = 0.0f;
    g_ekf_ctx.last_pos[2]     = 0.0f;
    g_ekf_ctx.last_vel[0]     = 0.0f;
    g_ekf_ctx.last_vel[1]     = 0.0f;
    g_ekf_ctx.last_vel[2]     = 0.0f;
    g_ekf_ctx.last_update_us  = 0U;
    g_ekf_ctx.initialized     = true;

    return AGENT_OK;
}

agent_status_t agent_fdir_detector_reset(void)
{
    return agent_fdir_detector_init();
}

/* ---------- Range check ---------- */
bool agent_fdir_check_range_i16(int16_t value, int16_t min_v, int16_t max_v)
{
    return (value < min_v || value > max_v) ? false : true;
}

/* ---------- Stuck check ---------- */
bool agent_fdir_check_stuck_i16(int16_t current,
                                                  int16_t previous,
                                                  uint32_t stuck_count,
                                                  uint32_t threshold)
{
    return (current == previous && stuck_count >= threshold) ? true : false;
}

/* ---------- Spike check (no abs() call) ---------- */
bool agent_fdir_check_spike_i16(int16_t current,
                                                  int16_t previous,
                                                  int16_t delta_max)
{
    int32_t delta = (int32_t)current - (int32_t)previous;
    if (delta < 0) {
        delta = -delta;
    }
    return (delta > (int32_t)delta_max) ? true : false;
}

/* ---------- Timeout check ---------- */
bool agent_fdir_check_timeout(uint32_t last_update_us, uint32_t timeout_us)
{
    return ((agent_time_now_us() - last_update_us) > timeout_us) ? true : false;
}

/* ---------- Dropout check ---------- */
bool agent_fdir_check_dropout(uint32_t dt_us, uint32_t dropout_us)
{
    return (dt_us > dropout_us) ? true : false;
}

/* ---------- Innovation gate ---------- */
bool agent_fdir_check_innovation(float innovation, float sigma, float gate)
{
    float abs_inno = (innovation < 0.0f) ? -innovation : innovation;
    float threshold = sigma * gate;
    return (abs_inno > threshold) ? true : false;
}

/* ---------- IMU detection ---------- */
static agent_status_t detect_imu(agent_fdir_fault_t *fault_out, uint32_t *reason_out)
{
    agent_sensor_sample_t sample;

    if (imu_get_sample(&sample) != AGENT_OK) {
        *fault_out  = AGENT_FDIR_FAULT_TIMEOUT;
        *reason_out = 0x01U;
        return AGENT_OK;
    }

    if (imu_health() == AGENT_HEALTH_FAIL) {
        *fault_out  = AGENT_FDIR_FAULT_TIMEOUT;
        *reason_out = 0x02U;
        return AGENT_OK;
    }

    if (!agent_fdir_check_range_i16(sample.ax, FDIR_RANGE_MIN, FDIR_RANGE_MAX) ||
        !agent_fdir_check_range_i16(sample.ay, FDIR_RANGE_MIN, FDIR_RANGE_MAX) ||
        !agent_fdir_check_range_i16(sample.az, FDIR_RANGE_MIN, FDIR_RANGE_MAX) ||
        !agent_fdir_check_range_i16(sample.gx, FDIR_RANGE_MIN, FDIR_RANGE_MAX) ||
        !agent_fdir_check_range_i16(sample.gy, FDIR_RANGE_MIN, FDIR_RANGE_MAX) ||
        !agent_fdir_check_range_i16(sample.gz, FDIR_RANGE_MIN, FDIR_RANGE_MAX)) {

        *fault_out  = AGENT_FDIR_FAULT_RANGE;
        *reason_out = 0x03U;
        return AGENT_OK;
    }

    if (g_imu_ctx.initialized) {

        if (agent_fdir_check_spike_i16(sample.ax, g_imu_ctx.last_ax, FDIR_SPIKE_DELTA) ||
            agent_fdir_check_spike_i16(sample.ay, g_imu_ctx.last_ay, FDIR_SPIKE_DELTA) ||
            agent_fdir_check_spike_i16(sample.az, g_imu_ctx.last_az, FDIR_SPIKE_DELTA)) {

            *fault_out  = AGENT_FDIR_FAULT_SPIKE;
            *reason_out = 0x04U;
            return AGENT_OK;
        }

        if (sample.ax == g_imu_ctx.last_ax &&
            sample.ay == g_imu_ctx.last_ay &&
            sample.az == g_imu_ctx.last_az &&
            sample.gx == g_imu_ctx.last_gx &&
            sample.gy == g_imu_ctx.last_gy &&
            sample.gz == g_imu_ctx.last_gz) {

            g_imu_ctx.stuck_count++;

            if (g_imu_ctx.stuck_count >= FDIR_STUCK_THRESHOLD) {
                *fault_out  = AGENT_FDIR_FAULT_STUCK;
                *reason_out = 0x05U;
                return AGENT_OK;
            }
        } else {
            g_imu_ctx.stuck_count = 0U;
        }

        uint32_t dt = agent_time_now_us() - g_imu_ctx.last_update_us;
        if (agent_fdir_check_dropout(dt, AGENT_DROPOUT_THRESHOLD_US)) {
            *fault_out  = AGENT_FDIR_FAULT_DROPOUT;
            *reason_out = 0x06U;
            return AGENT_OK;
        }
    }

    g_imu_ctx.last_ax        = sample.ax;
    g_imu_ctx.last_ay        = sample.ay;
    g_imu_ctx.last_az        = sample.az;
    g_imu_ctx.last_gx        = sample.gx;
    g_imu_ctx.last_gy        = sample.gy;
    g_imu_ctx.last_gz        = sample.gz;
    g_imu_ctx.last_update_us = agent_time_now_us();

    *fault_out  = AGENT_FDIR_FAULT_NONE;
    *reason_out = 0U;
    return AGENT_OK;
}

/* ---------- AI detection ---------- */
static agent_status_t detect_ai(agent_fdir_fault_t *fault_out, uint32_t *reason_out)
{
    if (ai_health() != AGENT_OK) {
        *fault_out  = AGENT_FDIR_FAULT_TIMEOUT;
        *reason_out = 0x10U;
        return AGENT_OK;
    }

    ai_status_t st = ai_status();

    if (st == AI_STATUS_ERROR) {
        *fault_out  = AGENT_FDIR_FAULT_UNKNOWN;
        *reason_out = 0x11U;
        return AGENT_OK;
    }

    if (st == AI_STATUS_RUNNING) {
        uint32_t dt = agent_time_now_us() - g_ai_ctx.last_update_us;
        if (agent_fdir_check_dropout(dt, AGENT_AI_MAX_LATENCY_US)) {
            *fault_out  = AGENT_FDIR_FAULT_TIMEOUT;
            *reason_out = 0x12U;
            return AGENT_OK;
        }
    }

    g_ai_ctx.last_update_us  = agent_time_now_us();
    g_ai_ctx.last_confidence = agent_confidence_get();

    *fault_out  = AGENT_FDIR_FAULT_NONE;
    *reason_out = 0U;
    return AGENT_OK;
}

/* ---------- EKF detection ---------- */
static agent_status_t detect_ekf(agent_fdir_fault_t *fault_out, uint32_t *reason_out)
{
    if (ekf_health() != AGENT_OK) {
        *fault_out  = AGENT_FDIR_FAULT_UNKNOWN;
        *reason_out = 0x20U;
        return AGENT_OK;
    }

    if (ekf_has_diverged()) {
        *fault_out  = AGENT_FDIR_FAULT_INNOVATION;
        *reason_out = 0x21U;
        return AGENT_OK;
    }

    agent_ekf_state_t state;
    if (ekf_get_state(&state) != AGENT_OK) {
        *fault_out  = AGENT_FDIR_FAULT_UNKNOWN;
        *reason_out = 0x22U;
        return AGENT_OK;
    }

    if (state.confidence < AGENT_CONFIDENCE_MIN) {
        *fault_out  = AGENT_FDIR_FAULT_DRIFT;
        *reason_out = 0x23U;
        return AGENT_OK;
    }

    g_ekf_ctx.last_pos[0]    = state.pos[0];
    g_ekf_ctx.last_pos[1]    = state.pos[1];
    g_ekf_ctx.last_pos[2]    = state.pos[2];
    g_ekf_ctx.last_vel[0]    = state.vel[0];
    g_ekf_ctx.last_vel[1]    = state.vel[1];
    g_ekf_ctx.last_vel[2]    = state.vel[2];
    g_ekf_ctx.last_update_us = agent_time_now_us();

    *fault_out  = AGENT_FDIR_FAULT_NONE;
    *reason_out = 0U;
    return AGENT_OK;
}

/* ---------- Main detect dispatch ---------- */
agent_status_t agent_fdir_detect(agent_fdir_sensor_t  sensor,
                                 agent_fdir_fault_t  *fault_out,
                                 uint32_t            *reason_out)
{
    if (fault_out == NULL || reason_out == NULL) {
        return AGENT_ERR;
    }

    *fault_out  = AGENT_FDIR_FAULT_NONE;
    *reason_out = 0U;

    switch (sensor) {
    case AGENT_FDIR_SENSOR_IMU:  return detect_imu(fault_out, reason_out);
    case AGENT_FDIR_SENSOR_AI:   return detect_ai(fault_out, reason_out);
    case AGENT_FDIR_SENSOR_EKF:  return detect_ekf(fault_out, reason_out);
    default:                     return AGENT_OK;
    }
}