/* ============================================================
 * HAL FLASH — ABSTRACTION LAYER
 * ============================================================
 * - Uniform interface over Zephyr flash_map
 * - Safe: verify erase/write alignment, bounds, CRC on demand
 * - Extensible: add partitions by name without code changes
 * - Idempotent: repeat erase is safe
 * - Production-grade: no direct HAL calls elsewhere
 * ============================================================ */

#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "validation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Logical Partition IDs
 * ============================================================ */
typedef enum {
    HAL_FLASH_PART_CONFIG     = 0,
    HAL_FLASH_PART_LOGS       = 1,
    HAL_FLASH_PART_CALIB      = 2,
    HAL_FLASH_PART_COUNT      = 3
} hal_flash_part_t;

/* ============================================================
 * Init
 * ============================================================ */
proj_status_t hal_flash_init(void);

/* ============================================================
 * Read / Write / Erase
 * ============================================================ */
proj_status_t hal_flash_read(hal_flash_part_t part,
                             uint32_t         offset,
                             void            *buf,
                             size_t           len);

proj_status_t hal_flash_write(hal_flash_part_t part,
                              uint32_t         offset,
                              const void      *buf,
                              size_t           len);

proj_status_t hal_flash_erase(hal_flash_part_t part);

/* ============================================================
 * Geometry Query
 * ============================================================ */
uint32_t hal_flash_size(hal_flash_part_t part);
uint32_t hal_flash_write_block_size(hal_flash_part_t part);
uint32_t hal_flash_erase_block_size(hal_flash_part_t part);
bool     hal_flash_is_ready(hal_flash_part_t part);

#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_H */