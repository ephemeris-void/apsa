/* ============================================================
 * HAL DMA — DIRECT MEMORY ACCESS ABSTRACTION
 * ============================================================
 * - Zephyr DMA driver backend
 * - SPI + UART DMA support
 * - Cache-coherent (uses hal_cache automatically)
 * - Completion callback support
 * - Timeout-bounded
 * - Production-grade: no direct DMA calls elsewhere
 * ============================================================ */

#ifndef HAL_DMA_H
#define HAL_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * DMA Channel IDs (logical)
 * ============================================================ */
typedef enum {
    HAL_DMA_SPI1_TX = 0,
    HAL_DMA_SPI1_RX = 1,
    HAL_DMA_UART3_TX = 2,
    HAL_DMA_UART3_RX = 3,
    HAL_DMA_COUNT    = 4
} hal_dma_channel_t;

/* ============================================================
 * Callback
 * ============================================================ */
typedef void (*hal_dma_callback_t)(hal_dma_channel_t ch, void *user_data);

/* ============================================================
 * Init
 * ============================================================ */
proj_status_t hal_dma_init(void);

/* ============================================================
 * Start Transfer
 * ============================================================
 * Return: PROJ_OK if accepted
 *         PROJ_ERR_BUSY if channel in use
 * ============================================================ */
proj_status_t hal_dma_start(hal_dma_channel_t   ch,
                            const void         *src,
                            void               *dst,
                            size_t              len,
                            hal_dma_callback_t  cb,
                            void               *user_data);

/* ============================================================
 * Wait for Completion (blocking, timeout-bounded)
 * ============================================================ */
proj_status_t hal_dma_wait(hal_dma_channel_t ch, uint32_t timeout_us);

/* ============================================================
 * Abort
 * ============================================================ */
proj_status_t hal_dma_abort(hal_dma_channel_t ch);

/* ============================================================
 * Query
 * ============================================================ */
bool     hal_dma_is_busy(hal_dma_channel_t ch);
uint32_t hal_dma_transfer_count(hal_dma_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* HAL_DMA_H */