/* ============================================================
 * HAL SPI — ABSTRACTION LAYER
 * ============================================================
 * - Zephyr SPI driver backend
 * - Thread-safe with mutex per bus
 * - Zero-copy transmit-receive
 * - Timeout-bounded operations
 * ============================================================ */

#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "project_config.h"
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- SPI Bus IDs ---------- */
typedef enum {
    HAL_SPI_BUS_IMU    = 0,
    HAL_SPI_BUS_COUNT  = 1
} hal_spi_bus_t;

/* ---------- Init ---------- */
proj_status_t hal_spi_init(void);

/* ---------- Full-Duplex Transfer (blocking) ---------- */
proj_status_t hal_spi_transceive(hal_spi_bus_t bus,
                                 const uint8_t *tx,
                                 uint8_t       *rx,
                                 size_t         len,
                                 uint32_t       timeout_us);

/* ---------- Write Only ---------- */
proj_status_t hal_spi_write(hal_spi_bus_t bus,
                            const uint8_t *tx,
                            size_t         len,
                            uint32_t       timeout_us);

/* ---------- Read Only ---------- */
proj_status_t hal_spi_read(hal_spi_bus_t bus,
                           uint8_t       *rx,
                           size_t         len,
                           uint32_t       timeout_us);

/* ---------- Register Access Helpers ---------- */
proj_status_t hal_spi_reg_read(hal_spi_bus_t bus,
                               uint8_t        reg,
                               uint8_t       *val,
                               uint32_t       timeout_us);

proj_status_t hal_spi_reg_write(hal_spi_bus_t bus,
                                uint8_t        reg,
                                uint8_t        val,
                                uint32_t       timeout_us);

proj_status_t hal_spi_burst_read(hal_spi_bus_t bus,
                                 uint8_t        reg,
                                 uint8_t       *buf,
                                 size_t         len,
                                 uint32_t       timeout_us);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H */