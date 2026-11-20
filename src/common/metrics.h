/* ============================================================
 * METRICS — RUNTIME PERFORMANCE TRACKING
 * ============================================================
 * - Lock-free counters and timers
 * - Per-module WCET, call count, error count
 * - Statistical: min / max / avg (EWMA)
 * - No dynamic allocation
 * - Production-grade observability
 * ============================================================ */

#ifndef METRICS_H
#define METRICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Metric IDs ---------- */
typedef enum {
    METRIC_SENSOR_READ      = 0,
    METRIC_AGENT_STEP       = 1,
    METRIC_EKF_UPDATE       = 2,
    METRIC_AI_INFERENCE     = 3,
    METRIC_CONTROL_UPDATE   = 4,
    METRIC_ROS_PUBLISH      = 5,
    METRIC_IPC_SEND         = 6,
    METRIC_IPC_RECV         = 7,
    METRIC_COUNT            = 8
} metric_id_t;

/* ---------- Metric Slot ---------- */
typedef struct __attribute__((aligned(32))) {
    volatile uint32_t  call_count;
    volatile uint32_t  error_count;
    volatile uint32_t  wcet_us;         /* worst-case execution time */
    volatile uint32_t  last_us;
    volatile uint32_t  total_us;
    volatile uint32_t  min_us;
    volatile uint32_t  ewma_us;         /* exponentially weighted moving avg */
    volatile uint32_t  deadline_miss;
    volatile uint32_t  reserved;
} metric_slot_t;

/* ---------- Global Metrics Block ---------- */
typedef struct __attribute__((aligned(32))) {
    metric_slot_t    slots[METRIC_COUNT];
    volatile uint32_t system_uptime_ms;
    volatile uint32_t system_loop_count;
    volatile uint32_t system_wcet_max_us;
    uint8_t          initialized;
    uint8_t          reserved[31];
} metrics_block_t;

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t metrics_init(void);

/* ============================================================
 * BEGIN / END — Simple timing
 * ============================================================ */
void metrics_begin(metric_id_t id);
void metrics_end(metric_id_t id);
void metrics_end_error(metric_id_t id);

/* ============================================================
 * QUERY
 * ============================================================ */
const metric_slot_t *metrics_get(metric_id_t id);
const metrics_block_t *metrics_block(void);

/* ============================================================
 * RESET
 * ============================================================ */
void metrics_reset_all(void);
void metrics_reset(metric_id_t id);

/* ============================================================
 * DUMP — Print all metrics to console
 * ============================================================ */
void metrics_dump(void);

/* ============================================================
 * SYSTEM COUNTERS
 * ============================================================ */
void metrics_inc_loop(void);
void metrics_set_wcet(uint32_t us);
void metrics_set_uptime(uint32_t ms);

/* ============================================================
 * CONVENIENCE MACROS
 * ============================================================ */
#define METRIC_SCOPE(id)                    \
    for (int _once = (metrics_begin(id), 1); \
         _once;                             \
         _once = (metrics_end(id), 0))

#define METRIC_SCOPE_ERROR(id)                    \
    for (int _once = (metrics_begin(id), 1);      \
         _once;                                    \
         _once = (metrics_end_error(id), 0))

#ifdef __cplusplus
}
#endif

#endif /* METRICS_H */