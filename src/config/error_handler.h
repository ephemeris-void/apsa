/* ============================================================
 * ERROR HANDLER — CENTRALIZED ERROR + FAULT LOG
 * ============================================================
 * - Thread-safe error recording
 * - Ring buffer of last N errors
 * - Callback hook for fatal escalation
 * - Zero dynamic allocation
 * - Production-grade: no silent failures
 * ============================================================ */

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "project_config.h"
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Error Record ---------- */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t        timestamp_us;
    proj_status_t   code;
    uint16_t        source_id;      /* module ID */
    uint16_t        line;
    uint32_t        context;        /* user-supplied */
    const char     *file;           /* pointer to static string */
    const char     *message;        /* pointer to static string */
} error_record_t;

/* ---------- Module IDs (for source_id) ---------- */
typedef enum {
    ERR_SRC_NONE        = 0,
    ERR_SRC_AGENT       = 1,
    ERR_SRC_EKF         = 2,
    ERR_SRC_AI          = 3,
    ERR_SRC_IMU         = 4,
    ERR_SRC_ROS         = 5,
    ERR_SRC_IPC         = 6,
    ERR_SRC_CONTROL     = 7,
    ERR_SRC_HEALTH      = 8,
    ERR_SRC_COMM        = 9,
    ERR_SRC_HAL         = 10,
    ERR_SRC_COUNT       = 11
} error_source_t;

/* ---------- Severity Level ---------- */
typedef enum {
    ERR_SEV_INFO        = 0,
    ERR_SEV_WARN        = 1,
    ERR_SEV_ERROR       = 2,
    ERR_SEV_CRITICAL    = 3,
    ERR_SEV_FATAL       = 4
} error_severity_t;

/* ---------- Fatal Callback (user override) ---------- */
typedef void (*error_fatal_callback_t)(const error_record_t *rec);

/* ---------- Public API ---------- */

/* Initialize error handler */
proj_status_t error_handler_init(void);

/* Record an error (thread-safe) */
void error_handler_record(proj_status_t code,
                          error_source_t source,
                          error_severity_t severity,
                          uint32_t context,
                          const char *file,
                          uint16_t line,
                          const char *message);

/* Query */
uint32_t error_handler_count(void);
uint32_t error_handler_count_by_severity(error_severity_t sev);
const error_record_t *error_handler_get(uint32_t index);

/* Latest error */
const error_record_t *error_handler_latest(void);

/* Clear log */
void error_handler_clear(void);

/* Dump to console (UART) */
void error_handler_dump(void);

/* Register fatal callback */
void error_handler_set_fatal_callback(error_fatal_callback_t cb);

/* ============================================================
 * MACROS — Use these everywhere
 * ============================================================ */

#define ERROR_RECORD(code, source, severity, context, msg)          \
    error_handler_record((code), (source), (severity),              \
                         (context), __FILE__, __LINE__, (msg))

#define ERROR_LOG_INFO(source, msg)                                 \
    ERROR_RECORD(PROJ_OK, (source), ERR_SEV_INFO, 0, (msg))

#define ERROR_LOG_WARN(code, source, msg)                           \
    ERROR_RECORD((code), (source), ERR_SEV_WARN, 0, (msg))

#define ERROR_LOG_ERROR(code, source, msg)                          \
    ERROR_RECORD((code), (source), ERR_SEV_ERROR, 0, (msg))

#define ERROR_LOG_CRITICAL(code, source, msg)                       \
    ERROR_RECORD((code), (source), ERR_SEV_CRITICAL, 0, (msg))

#define ERROR_LOG_FATAL(code, source, msg)                          \
    ERROR_RECORD((code), (source), ERR_SEV_FATAL, 0, (msg))

/* Convenience — return + log */
#define ERROR_RETURN(code, source, msg)                             \
    do {                                                            \
        ERROR_LOG_ERROR((code), (source), (msg));                   \
        return (code);                                              \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* ERROR_HANDLER_H */