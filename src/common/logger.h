/* ============================================================
 * LOGGER — STRUCTURED, RATE-LIMITED, NON-BLOCKING
 * ============================================================
 * - Level-based filtering (compile-time + runtime)
 * - Non-blocking write (drops when UART busy)
 * - Rate-limited per tag
 * - Static string literals only (no malloc)
 * - Thread-safe
 * - Binary log support (for high-speed telemetry)
 * - Production-grade observability
 * ============================================================ */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Log Levels
 * ============================================================ */
typedef enum {
    LOG_LEVEL_NONE   = 0,
    LOG_LEVEL_ERROR  = 1,
    LOG_LEVEL_WARN   = 2,
    LOG_LEVEL_INFO   = 3,
    LOG_LEVEL_DEBUG  = 4,
    LOG_LEVEL_TRACE  = 5
} log_level_t;

/* ============================================================
 * Log Tags (module identifiers)
 * ============================================================ */
typedef enum {
    LOG_TAG_MAIN      = 0,
    LOG_TAG_AGENT     = 1,
    LOG_TAG_EKF       = 2,
    LOG_TAG_AI        = 3,
    LOG_TAG_IMU       = 4,
    LOG_TAG_ROS       = 5,
    LOG_TAG_IPC       = 6,
    LOG_TAG_CONTROL   = 7,
    LOG_TAG_HEALTH    = 8,
    LOG_TAG_PIPELINE  = 9,
    LOG_TAG_METRICS   = 10,
    LOG_TAG_COUNT     = 11
} log_tag_t;

/* ============================================================
 * Init
 * ============================================================ */
proj_status_t logger_init(log_level_t global_level);

/* ============================================================
 * Runtime level control
 * ============================================================ */
void        logger_set_level(log_level_t level);
log_level_t logger_get_level(void);

void        logger_set_tag_level(log_tag_t tag, log_level_t level);
log_level_t logger_get_tag_level(log_tag_t tag);

/* ============================================================
 * Core log function (use macros below)
 * ============================================================ */
void logger_log(log_level_t  level,
                log_tag_t    tag,
                const char  *file,
                uint16_t     line,
                const char  *fmt,
                ...);

/* ============================================================
 * Binary log (high-speed telemetry)
 * ============================================================ */
void logger_binary(log_tag_t   tag,
                   const void *data,
                   size_t      len);

/* ============================================================
 * Rate-limited log — max N per second per tag
 * ============================================================ */
void logger_log_ratelimited(log_level_t  level,
                            log_tag_t    tag,
                            uint32_t     max_per_sec,
                            const char  *file,
                            uint16_t     line,
                            const char  *fmt,
                            ...);

/* ============================================================
 * Dump stats
 * ============================================================ */
void logger_dump_stats(void);

/* ============================================================
 * MACROS
 * ============================================================ */

#define LOG_E(tag, ...)  logger_log(LOG_LEVEL_ERROR, (tag), __FILE__, __LINE__, __VA_ARGS__)
#define LOG_W(tag, ...)  logger_log(LOG_LEVEL_WARN,  (tag), __FILE__, __LINE__, __VA_ARGS__)
#define LOG_I(tag, ...)  logger_log(LOG_LEVEL_INFO,  (tag), __FILE__, __LINE__, __VA_ARGS__)
#define LOG_D(tag, ...)  logger_log(LOG_LEVEL_DEBUG, (tag), __FILE__, __LINE__, __VA_ARGS__)
#define LOG_T(tag, ...)  logger_log(LOG_LEVEL_TRACE, (tag), __FILE__, __LINE__, __VA_ARGS__)

#define LOG_RL_E(tag, mps, ...) logger_log_ratelimited(LOG_LEVEL_ERROR, (tag), (mps), __FILE__, __LINE__, __VA_ARGS__)
#define LOG_RL_W(tag, mps, ...) logger_log_ratelimited(LOG_LEVEL_WARN,  (tag), (mps), __FILE__, __LINE__, __VA_ARGS__)
#define LOG_RL_I(tag, mps, ...) logger_log_ratelimited(LOG_LEVEL_INFO,  (tag), (mps), __FILE__, __LINE__, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */