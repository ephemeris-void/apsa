#ifndef AGENT_FDIR_DETECTOR_H
#define AGENT_FDIR_DETECTOR_H

#include "agent_fdir.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT FDIR DETECTOR — FROZEN
 * ============================================================
 * Fault detection primitives: range, stuck, spike, dropout,
 * timeout, innovation gate.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 60 µs max per detect.
 * ============================================================ */

agent_status_t     agent_fdir_detector_init(void);
agent_status_t     agent_fdir_detector_reset(void);

agent_status_t     agent_fdir_detect(agent_fdir_sensor_t  sensor,
                                     agent_fdir_fault_t  *fault_out,
                                     uint32_t            *reason_out);

bool               agent_fdir_check_range_i16(int16_t value,
                                              int16_t min_v,
                                              int16_t max_v);

bool               agent_fdir_check_stuck_i16(int16_t current,
                                              int16_t previous,
                                              uint32_t stuck_count,
                                              uint32_t threshold);

bool               agent_fdir_check_spike_i16(int16_t current,
                                              int16_t previous,
                                              int16_t delta_max);

bool               agent_fdir_check_timeout(uint32_t last_update_us,
                                            uint32_t timeout_us);

bool               agent_fdir_check_dropout(uint32_t dt_us,
                                            uint32_t dropout_us);

bool               agent_fdir_check_innovation(float    innovation,
                                               float    sigma,
                                               float    gate);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_FDIR_DETECTOR_H */