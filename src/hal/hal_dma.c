/* ============================================================
 * HAL DMA — STUB IMPLEMENTATION
 * ============================================================
 * DMA disabled due to Zephyr H7 DMA driver conflict.
 * All functions are no-op/error stubs.
 *
 * If DMA is needed in future:
 *   - Enable CONFIG_DMA=y in prj.conf
 *   - Add dma1 devicetree overlay
 *   - Implement actual DMA transfers
 * ============================================================ */

#include "hal_dma.h"
#include "error_handler.h"
#include <zephyr/kernel.h>

proj_status_t hal_dma_init(void)
{
    /* DMA disabled — no-op */
    return PROJ_OK;
}

proj_status_t hal_dma_start(hal_dma_channel_t   ch,
                            const void         *src,
                            void               *dst,
                            size_t              len,
                            hal_dma_callback_t  cb,
                            void               *user_data)
{
    ARG_UNUSED(ch);
    ARG_UNUSED(src);
    ARG_UNUSED(dst);
    ARG_UNUSED(len);
    ARG_UNUSED(cb);
    ARG_UNUSED(user_data);
    return PROJ_ERR;   /* DMA not available */
}

proj_status_t hal_dma_wait(hal_dma_channel_t ch, uint32_t timeout_us)
{
    ARG_UNUSED(ch);
    ARG_UNUSED(timeout_us);
    return PROJ_ERR;
}

proj_status_t hal_dma_abort(hal_dma_channel_t ch)
{
    ARG_UNUSED(ch);
    return PROJ_OK;
}

bool hal_dma_is_busy(hal_dma_channel_t ch)
{
    ARG_UNUSED(ch);
    return false;
}

uint32_t hal_dma_transfer_count(hal_dma_channel_t ch)
{
    ARG_UNUSED(ch);
    return 0U;
}
