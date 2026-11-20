/* ============================================================
 * HAL CACHE — ABSTRACTION LAYER
 * ============================================================
 * - D-Cache / I-Cache management for STM32H7
 * - ARM CMSIS backend
 * - Coherency helpers for DMA + shared memory
 * - No-op when caches disabled
 * - Thread-safe (idempotent operations)
 * ============================================================ */

#ifndef HAL_CACHE_H
#define HAL_CACHE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "project_config.h"
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Init ---------- */
proj_status_t hal_cache_init(void);

/* ---------- D-Cache: Flush (write-back to memory) ---------- */
void hal_dcache_flush(const void *addr, size_t size);

/* ---------- D-Cache: Invalidate (drop local copy) ---------- */
void hal_dcache_invalidate(const void *addr, size_t size);

/* ---------- D-Cache: Flush + Invalidate ---------- */
void hal_dcache_flush_invalidate(const void *addr, size_t size);

/* ---------- Alignment Helpers ---------- */
size_t hal_cache_line_size(void);

static inline bool hal_cache_is_aligned(const void *addr)
{
    uintptr_t a = (uintptr_t)addr;
    size_t line = hal_cache_line_size();
    if (line == 0U) return true;
    return (a % line) == 0U;
}

/* ---------- Status ---------- */
bool hal_dcache_enabled(void);
bool hal_icache_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CACHE_H */