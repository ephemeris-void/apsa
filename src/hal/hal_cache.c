/* ============================================================
 * HAL CACHE — IMPLEMENTATION
 * ============================================================ */

#include "hal_cache.h"

/* ARM CMSIS */
#include "stm32h7xx.h"

/* ---------- Constants ---------- */
#define CACHE_LINE_SIZE_BYTES   32U

/* ---------- Internal State ---------- */
static bool g_initialized = false;
static bool g_dcache_enabled = false;
static bool g_icache_enabled = false;

/* ============================================================
 * INIT — Query current cache state
 * ============================================================ */
proj_status_t hal_cache_init(void)
{
    if (g_initialized) {
        return PROJ_OK;
    }

    /* Query current state from SCB */
    g_icache_enabled = ((SCB->CCR & SCB_CCR_IC_Msk) != 0U);
    g_dcache_enabled = ((SCB->CCR & SCB_CCR_DC_Msk) != 0U);

    g_initialized = true;
    return PROJ_OK;
}

/* ============================================================
 * CACHE LINE SIZE
 * ============================================================ */
size_t hal_cache_line_size(void)
{
    return CACHE_LINE_SIZE_BYTES;
}

/* ============================================================
 * STATUS
 * ============================================================ */
bool hal_dcache_enabled(void)
{
    return g_dcache_enabled;
}

bool hal_icache_enabled(void)
{
    return g_icache_enabled;
}

/* ============================================================
 * D-CACHE FLUSH
 * ============================================================ */
void hal_dcache_flush(const void *addr, size_t size)
{
    if (addr == NULL || size == 0U) {
        return;
    }

    if (!g_dcache_enabled) {
        return;
    }

    uintptr_t start = (uintptr_t)addr;
    uintptr_t end   = start + size;

    start &= ~((uintptr_t)CACHE_LINE_SIZE_BYTES - 1U);
    end = (end + CACHE_LINE_SIZE_BYTES - 1U) &
          ~((uintptr_t)CACHE_LINE_SIZE_BYTES - 1U);

    SCB_CleanDCache_by_Addr((uint32_t *)start,
                            (int32_t)(end - start));
}

/* ============================================================
 * D-CACHE INVALIDATE
 * ============================================================ */
void hal_dcache_invalidate(const void *addr, size_t size)
{
    if (addr == NULL || size == 0U) {
        return;
    }

    if (!g_dcache_enabled) {
        return;
    }

    uintptr_t start = (uintptr_t)addr;
    uintptr_t end   = start + size;

    start &= ~((uintptr_t)CACHE_LINE_SIZE_BYTES - 1U);
    end = (end + CACHE_LINE_SIZE_BYTES - 1U) &
          ~((uintptr_t)CACHE_LINE_SIZE_BYTES - 1U);

    SCB_InvalidateDCache_by_Addr((uint32_t *)start,
                                 (int32_t)(end - start));
}

/* ============================================================
 * D-CACHE FLUSH + INVALIDATE
 * ============================================================ */
void hal_dcache_flush_invalidate(const void *addr, size_t size)
{
    if (addr == NULL || size == 0U) {
        return;
    }

    if (!g_dcache_enabled) {
        return;
    }

    uintptr_t start = (uintptr_t)addr;
    uintptr_t end   = start + size;

    start &= ~((uintptr_t)CACHE_LINE_SIZE_BYTES - 1U);
    end = (end + CACHE_LINE_SIZE_BYTES - 1U) &
          ~((uintptr_t)CACHE_LINE_SIZE_BYTES - 1U);

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start,
                                      (int32_t)(end - start));
}