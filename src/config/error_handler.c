/* ============================================================
 * ERROR HANDLER — IMPLEMENTATION
 * ============================================================
 * Lock-free single-writer ring buffer.
 * Thread-safe for single writer, multiple readers.
 * ============================================================ */

#include "error_handler.h"
#include <string.h>
#include <stdio.h>

/* Zephyr dependencies */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/* ---------- Configuration ---------- */
#ifndef CFG_ERROR_LOG_DEPTH
#define CFG_ERROR_LOG_DEPTH     64U
#endif

/* ---------- Internal State ---------- */
static error_record_t              g_error_log[CFG_ERROR_LOG_DEPTH];
static atomic_t                    g_error_head = ATOMIC_INIT(0);
static atomic_t                    g_error_count = ATOMIC_INIT(0);
static atomic_t                    g_severity_count[5];
static error_record_t              g_error_latest;
static bool                        g_initialized = false;
static error_fatal_callback_t      g_fatal_cb = NULL;

/* Zephyr spinlock for multi-writer safety */
static struct k_spinlock           g_lock;

/* ---------- INIT ---------- */
proj_status_t error_handler_init(void)
{
    memset(g_error_log, 0, sizeof(g_error_log));
    memset(&g_error_latest, 0, sizeof(g_error_latest));

    atomic_set(&g_error_head, 0);
    atomic_set(&g_error_count, 0);

    for (int i = 0; i < 5; i++) {
        atomic_set(&g_severity_count[i], 0);
    }

    g_fatal_cb = NULL;
    g_initialized = true;

    return PROJ_OK;
}

/* ---------- RECORD ---------- */
void error_handler_record(proj_status_t code,
                          error_source_t source,
                          error_severity_t severity,
                          uint32_t context,
                          const char *file,
                          uint16_t line,
                          const char *message)
{
    if (!g_initialized) {
        return;
    }

    if (severity >= ERR_SEV_FATAL) {
        /* Fatal — do not return to caller */
        /* Just log and let callback decide */
    }

    k_spinlock_key_t key = k_spin_lock(&g_lock);

    uint32_t idx = (uint32_t)atomic_get(&g_error_head);
    error_record_t *rec = &g_error_log[idx % CFG_ERROR_LOG_DEPTH];

    rec->timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
    rec->code         = code;
    rec->source_id    = (uint16_t)source;
    rec->line         = line;
    rec->context      = context;
    rec->file         = file;
    rec->message      = message;

    /* Update latest */
    memcpy(&g_error_latest, rec, sizeof(error_record_t));

    /* Advance head */
    atomic_inc(&g_error_head);
    atomic_inc(&g_error_count);

    if (severity < 5) {
        atomic_inc(&g_severity_count[severity]);
    }

    k_spin_unlock(&g_lock, key);

    /* Fatal — invoke callback (outside lock) */
    if (severity == ERR_SEV_FATAL && g_fatal_cb != NULL) {
        g_fatal_cb(rec);
    }

    /* Also print to console if ERROR or above */
    if (severity >= ERR_SEV_ERROR) {
        printk("[ERR] src=%u code=%d line=%u: %s\n",
               (unsigned)source, (int)code, (unsigned)line,
               (message != NULL) ? message : "(null)");
    }
}

/* ---------- QUERY ---------- */
uint32_t error_handler_count(void)
{
    return (uint32_t)atomic_get(&g_error_count);
}

uint32_t error_handler_count_by_severity(error_severity_t sev)
{
    if (sev >= 5) {
        return 0;
    }
    return (uint32_t)atomic_get(&g_severity_count[sev]);
}

const error_record_t *error_handler_get(uint32_t index)
{
    uint32_t count = (uint32_t)atomic_get(&g_error_count);
    if (index >= count) {
        return NULL;
    }

    /* Index from head backwards */
    uint32_t head = (uint32_t)atomic_get(&g_error_head);
    uint32_t back = count - 1 - index;

    if (back >= CFG_ERROR_LOG_DEPTH) {
        return NULL;
    }

    uint32_t pos = (head - 1 - back) % CFG_ERROR_LOG_DEPTH;
    return &g_error_log[pos];
}

const error_record_t *error_handler_latest(void)
{
    return &g_error_latest;
}

/* ---------- CLEAR ---------- */
void error_handler_clear(void)
{
    k_spinlock_key_t key = k_spin_lock(&g_lock);

    atomic_set(&g_error_head, 0);
    atomic_set(&g_error_count, 0);
    memset(&g_error_latest, 0, sizeof(g_error_latest));

    for (int i = 0; i < 5; i++) {
        atomic_set(&g_severity_count[i], 0);
    }

    k_spin_unlock(&g_lock, key);
}

/* ---------- DUMP ---------- */
void error_handler_dump(void)
{
    uint32_t total = error_handler_count();
    uint32_t shown = (total < 10) ? total : 10;

    printk("\n=== ERROR LOG DUMP (last %u of %u) ===\n",
           (unsigned)shown, (unsigned)total);

    for (uint32_t i = 0; i < shown; i++) {
        const error_record_t *rec = error_handler_get(i);
        if (rec == NULL) break;

        printk("[%u] t=%uus src=%u code=%d line=%u: %s\n",
               (unsigned)i,
               (unsigned)rec->timestamp_us,
               (unsigned)rec->source_id,
               (int)rec->code,
               (unsigned)rec->line,
               (rec->message != NULL) ? rec->message : "(null)");
    }

    printk("=== END ERROR LOG ===\n\n");
}

/* ---------- FATAL CALLBACK ---------- */
void error_handler_set_fatal_callback(error_fatal_callback_t cb)
{
    g_fatal_cb = cb;
}

/* ---------- WEAK LOG FAIL (override for validation) ---------- */
__attribute__((weak))
void validation_log_fail(const char *file, int line, const char *msg)
{
    ERROR_RECORD(PROJ_ERR, ERR_SRC_NONE, ERR_SEV_ERROR, 0,
                 (msg != NULL) ? msg : "validation_fail");
    (void)file;
    (void)line;
}