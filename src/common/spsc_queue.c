/* ============================================================
 * SPSC QUEUE — IMPLEMENTATION
 * ============================================================ */

#include "spsc_queue.h"
#include <string.h>

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t spsc_init(spsc_queue_t *q,
                        void        **storage,
                        uint32_t      capacity)
{
    VALIDATE_NOT_NULL(q);
    VALIDATE_NOT_NULL(storage);
    VALIDATE_RETURN(capacity < 2U, PROJ_ERR_INVALID);
    VALIDATE_RETURN(!SPSC_IS_POW2(capacity), PROJ_ERR_INVALID);

    q->slots          = storage;
    q->capacity       = capacity;
    q->mask           = capacity - 1U;
    q->head           = 0U;
    q->tail           = 0U;
    q->push_count     = 0U;
    q->pop_count      = 0U;
    q->overflow_count = 0U;
    q->empty_count    = 0U;
    q->initialized    = true;

    memset(storage, 0, sizeof(void *) * (size_t)capacity);

    return PROJ_OK;
}

/* ============================================================
 * PUSH
 * ============================================================ */
proj_status_t spsc_push(spsc_queue_t *q, void *ptr)
{
    VALIDATE_NOT_NULL(q);
    VALIDATE_NOT_NULL(ptr);
    VALIDATE_INITIALIZED(q->initialized);

    uint32_t head = q->head;
    uint32_t tail = q->tail;

    if (((head + 1U) & q->mask) == (tail & q->mask)) {
        q->overflow_count++;
        return PROJ_ERR_BUSY;
    }

    q->slots[head & q->mask] = ptr;

    __atomic_thread_fence(__ATOMIC_RELEASE);

    q->head = head + 1U;
    q->push_count++;

    return PROJ_OK;
}

/* ============================================================
 * POP
 * ============================================================ */
proj_status_t spsc_pop(spsc_queue_t *q, void **ptr_out)
{
    VALIDATE_NOT_NULL(q);
    VALIDATE_NOT_NULL(ptr_out);
    VALIDATE_INITIALIZED(q->initialized);

    uint32_t head = q->head;
    uint32_t tail = q->tail;

    if ((tail & q->mask) == (head & q->mask)) {
        q->empty_count++;
        return PROJ_ERR_NOT_FOUND;
    }

    __atomic_thread_fence(__ATOMIC_ACQUIRE);

    *ptr_out = q->slots[tail & q->mask];

    q->tail = tail + 1U;
    q->pop_count++;

    return PROJ_OK;
}

/* ============================================================
 * QUERY
 * ============================================================ */
uint32_t spsc_count(const spsc_queue_t *q)
{
    if (q == NULL || !q->initialized) {
        return 0U;
    }
    return (q->head - q->tail) & q->mask;
}

uint32_t spsc_capacity(const spsc_queue_t *q)
{
    if (q == NULL || !q->initialized) {
        return 0U;
    }
    return q->capacity - 1U;
}

bool spsc_is_empty(const spsc_queue_t *q)
{
    return (spsc_count(q) == 0U);
}

bool spsc_is_full(const spsc_queue_t *q)
{
    if (q == NULL || !q->initialized) {
        return true;
    }
    return (((q->head + 1U) & q->mask) == (q->tail & q->mask));
}

uint32_t spsc_overflow(const spsc_queue_t *q)
{
    if (q == NULL) {
        return 0U;
    }
    return q->overflow_count;
}

uint32_t spsc_underflow(const spsc_queue_t *q)
{
    if (q == NULL) {
        return 0U;
    }
    return q->empty_count;
}

/* ============================================================
 * RESET
 * ============================================================ */
void spsc_reset(spsc_queue_t *q)
{
    if (q == NULL || !q->initialized) {
        return;
    }
    q->head           = 0U;
    q->tail           = 0U;
    q->push_count     = 0U;
    q->pop_count      = 0U;
    q->overflow_count = 0U;
    q->empty_count    = 0U;
}