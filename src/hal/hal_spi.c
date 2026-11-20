/* ============================================================
 * HAL SPI — IMPLEMENTATION (Zephyr)
 * ============================================================
 * - Zephyr SPI driver backend
 * - Thread-safe with mutex per bus
 * - CS auto-managed by Zephyr (via cs-gpios in app.overlay)
 * - ICM-42688-P compatible (R/W bit handling)
 * - SPI Mode 0 (CPOL=0, CPHA=0)
 * - Frequency 1 MHz
 * ============================================================ */

#include "hal_spi.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

/* Project HAL */
#include "hal_gpio.h"
#include "hal_time.h"

LOG_MODULE_REGISTER(hal_spi, LOG_LEVEL_INF);

/* ============================================================
 * Bus Context
 * ============================================================ */
typedef struct {
    const struct device *dev;
    struct k_mutex       lock;
    bool                 initialized;
} hal_spi_ctx_t;

static hal_spi_ctx_t g_spi_buses[HAL_SPI_BUS_COUNT];

/* ============================================================
 * SPI Config — Mode 0, 1 MHz
 * ============================================================ */
static struct spi_config g_spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_OP_MODE_MASTER
               | SPI_WORD_SET(8)
               | SPI_TRANSFER_MSB
               | SPI_LINES_SINGLE,
    .slave     = 0,
    .cs        = {
        .gpio  = { 0 },
        .delay = 0,
    },
};

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_spi_init(void)
{
    g_spi_buses[HAL_SPI_BUS_IMU].dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

    if (!device_is_ready(g_spi_buses[HAL_SPI_BUS_IMU].dev)) {
        LOG_ERR("SPI1 not ready");
        return PROJ_ERR_HW;
    }

    /* Fetch CS from devicetree */
    g_spi_cfg.cs.gpio = (struct gpio_dt_spec)
        GPIO_DT_SPEC_GET(DT_NODELABEL(spi1), cs_gpios);
    g_spi_cfg.cs.delay = 0;

    if (!device_is_ready(g_spi_cfg.cs.gpio.port)) {
        LOG_ERR("CS GPIO not ready");
        return PROJ_ERR_HW;
    }

    k_mutex_init(&g_spi_buses[HAL_SPI_BUS_IMU].lock);
    g_spi_buses[HAL_SPI_BUS_IMU].initialized = true;

    LOG_INF("SPI1 initialized @ 1 MHz, Mode 0, CS=PD%d",
            g_spi_cfg.cs.gpio.pin);
    return PROJ_OK;
}

/* ============================================================
 * TRANSCEIVE
 * ============================================================ */
proj_status_t hal_spi_transceive(hal_spi_bus_t bus,
                                 const uint8_t *tx,
                                 uint8_t       *rx,
                                 size_t         len,
                                 uint32_t       timeout_us)
{
    if (bus >= HAL_SPI_BUS_COUNT) return PROJ_ERR_INVALID;
    if (tx == NULL && rx == NULL) return PROJ_ERR_INVALID;

    hal_spi_ctx_t *ctx = &g_spi_buses[bus];
    if (!ctx->initialized) return PROJ_ERR_NOT_INIT;

    ARG_UNUSED(timeout_us);

    struct spi_buf tx_buf = { .buf = (void *)tx, .len = len };
    struct spi_buf rx_buf = { .buf = rx, .len = len };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    k_mutex_lock(&ctx->lock, K_FOREVER);
    int rc = spi_transceive(ctx->dev, &g_spi_cfg, &tx_set, &rx_set);
    k_mutex_unlock(&ctx->lock);

    if (rc != 0) {
        LOG_ERR("SPI transceive failed: %d", rc);
        return PROJ_ERR_HW;
    }

    return PROJ_OK;
}

/* ============================================================
 * WRITE
 * ============================================================ */
proj_status_t hal_spi_write(hal_spi_bus_t bus,
                            const uint8_t *tx,
                            size_t         len,
                            uint32_t       timeout_us)
{
    if (bus >= HAL_SPI_BUS_COUNT) return PROJ_ERR_INVALID;
    if (tx == NULL) return PROJ_ERR_INVALID;

    hal_spi_ctx_t *ctx = &g_spi_buses[bus];
    if (!ctx->initialized) return PROJ_ERR_NOT_INIT;

    ARG_UNUSED(timeout_us);

    struct spi_buf tx_buf = { .buf = (void *)tx, .len = len };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

    k_mutex_lock(&ctx->lock, K_FOREVER);
    int rc = spi_write(ctx->dev, &g_spi_cfg, &tx_set);
    k_mutex_unlock(&ctx->lock);

    if (rc != 0) return PROJ_ERR_HW;
    return PROJ_OK;
}

/* ============================================================
 * READ
 * ============================================================ */
proj_status_t hal_spi_read(hal_spi_bus_t bus,
                           uint8_t       *rx,
                           size_t         len,
                           uint32_t       timeout_us)
{
    if (bus >= HAL_SPI_BUS_COUNT) return PROJ_ERR_INVALID;
    if (rx == NULL) return PROJ_ERR_INVALID;

    hal_spi_ctx_t *ctx = &g_spi_buses[bus];
    if (!ctx->initialized) return PROJ_ERR_NOT_INIT;

    ARG_UNUSED(timeout_us);

    struct spi_buf rx_buf = { .buf = rx, .len = len };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    k_mutex_lock(&ctx->lock, K_FOREVER);
    int rc = spi_read(ctx->dev, &g_spi_cfg, &rx_set);
    k_mutex_unlock(&ctx->lock);

    if (rc != 0) return PROJ_ERR_HW;
    return PROJ_OK;
}

/* ============================================================
 * REG READ (ICM-42688-P: reg | 0x80)
 * ============================================================ */
proj_status_t hal_spi_reg_read(hal_spi_bus_t bus,
                               uint8_t        reg,
                               uint8_t       *val,
                               uint32_t       timeout_us)
{
    if (val == NULL) return PROJ_ERR_INVALID;

    uint8_t tx_buf[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx_buf[2] = { 0, 0 };

    proj_status_t st = hal_spi_transceive(bus, tx_buf, rx_buf, 2, timeout_us);

    if (st != PROJ_OK) {
        *val = 0xFF;
        return st;
    }

    *val = rx_buf[1];
    return PROJ_OK;
}

/* ============================================================
 * REG WRITE (ICM-42688-P: reg & 0x7F)
 * ============================================================ */
proj_status_t hal_spi_reg_write(hal_spi_bus_t bus,
                                uint8_t        reg,
                                uint8_t        val,
                                uint32_t       timeout_us)
{
    uint8_t tx_buf[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx_buf[2] = { 0, 0 };

    return hal_spi_transceive(bus, tx_buf, rx_buf, 2, timeout_us);
}

/* ============================================================
 * BURST READ (max 14 bytes for ICM-42688-P)
 * ============================================================ */
proj_status_t hal_spi_burst_read(hal_spi_bus_t bus,
                                 uint8_t        reg,
                                 uint8_t       *buf,
                                 size_t         len,
                                 uint32_t       timeout_us)
{
    if (buf == NULL || len == 0) return PROJ_ERR_INVALID;
    if (len > 14) return PROJ_ERR_INVALID;

    uint8_t tx_buf[1 + 14] = { 0 };
    uint8_t rx_buf[1 + 14] = { 0 };

    tx_buf[0] = (uint8_t)(reg | 0x80);

    proj_status_t st = hal_spi_transceive(bus, tx_buf, rx_buf, 1 + len, timeout_us);

    if (st != PROJ_OK) return st;

    for (size_t i = 0; i < len; i++) {
        buf[i] = rx_buf[i + 1];
    }

    return PROJ_OK;
}