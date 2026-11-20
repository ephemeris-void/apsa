/* ============================================================
 * SPSC QUEUE — LOCK-FREE SINGLE-PRODUCER SINGLE-CONSUMER
 * ============================================================
 * Zero-copy variant of ring buffer for RTOS threads.
 * - Power-of-2 capacity
 * - Pointer-based (avoids memcpy for large payloads)
 * - Cache-line padded to prevent false sharing
 * - Bounded overflow tracking
 * - Use when payload > 32 bytes
 * - Production-grade: no locks in steady state
 * ============================================================ */

#ifndef SPSC_QUEUE_H
#define SPSC_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPSC_IS_POW2(x)  (((x) != 0U) && (((x) & ((x) - 1U)) == 0U))

/* ============================================================
 * SPSC Queue
 * ============================================================ */
typedef struct __attribute__((aligned(64))) {
    void           **slots;             /* caller-supplied pointer array */
    uint32_t         capacity;          /* must be power of 2 */
    uint32_t         mask;

    /* Producer cache line */
    volatile uint32_t head __attribute__((aligned(64)));

    /* Consumer cache line */
    volatile uint32_t tail __attribute__((aligned(64)));

    /* Stats — separate cache line */
    volatile uint32_t push_count __attribute__((aligned(64)));
    volatile uint32_t pop_count;
    volatile uint32_t overflow_count;
    volatile uint32_t empty_count;

    bool             initialized;
    uint8_t          reserved[63];
} spsc_queue_t;

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t spsc_init(spsc_queue_t *q,
                        void        **storage,
                        uint32_t      capacity);

/* ============================================================
 * PUSH — Producer side
 * ============================================================ */
proj_status_t spsc_push(spsc_queue_t *q, void *ptr);

/* ============================================================
 * POP — Consumer side
 * ============================================================ */
proj_status_t spsc_pop(spsc_queue_t *q, void **ptr_out);

/* ============================================================
 * QUERY
 * ============================================================ */
uint32_t spsc_count(const spsc_queue_t *q);
uint32_t spsc_capacity(const spsc_queue_t *q);
bool     spsc_is_empty(const spsc_queue_t *q);
bool     spsc_is_full(const spsc_queue_t *q);
uint32_t spsc_overflow(const spsc_queue_t *q);
uint32_t spsc_underflow(const spsc_queue_t *q);

/* ============================================================
 * RESET
 * ============================================================ */
void spsc_reset(spsc_queue_t *q);

#ifdef __cplusplus
}
#endif

#endif /* SPSC_QUEUE_H */