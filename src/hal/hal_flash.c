/* ============================================================
 * HAL FLASH — IMPLEMENTATION (Zephyr)
 * ============================================================ */

#include "hal_flash.h"

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <string.h>

/* ============================================================
 * Partition Map
 * ============================================================ */
typedef struct {
    const char          *label;
    const struct device *dev;
    off_t                offset;
    size_t               size;
    bool                 ready;
} hal_flash_map_t;

static hal_flash_map_t g_flash_map[HAL_FLASH_PART_COUNT];

/* ============================================================
 * Internal: bind partition by label (currently unused — kept
 * for future dynamic partition support)
 * ============================================================ */
__attribute__((unused))
static proj_status_t bind_partition(hal_flash_part_t part, const char *label)
{
    if (part >= HAL_FLASH_PART_COUNT || label == NULL) {
        return PROJ_ERR_INVALID;
    }

    hal_flash_map_t *m = &g_flash_map[part];
    m->label = label;
    return PROJ_OK;
}

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t hal_flash_init(void)
{
    /* Partition setup — uses compile-time FIXED_PARTITION macros.
     * If a partition doesn't exist in devicetree, it's marked not ready. */

#if FIXED_PARTITION_EXISTS(config_storage)
    g_flash_map[HAL_FLASH_PART_CONFIG].dev    =
        FIXED_PARTITION_DEVICE(config_storage);
    g_flash_map[HAL_FLASH_PART_CONFIG].offset =
        FIXED_PARTITION_OFFSET(config_storage);
    g_flash_map[HAL_FLASH_PART_CONFIG].size   =
        FIXED_PARTITION_SIZE(config_storage);
    g_flash_map[HAL_FLASH_PART_CONFIG].ready  =
        device_is_ready(g_flash_map[HAL_FLASH_PART_CONFIG].dev);
#else
    g_flash_map[HAL_FLASH_PART_CONFIG].ready = false;
#endif

#if FIXED_PARTITION_EXISTS(logs_storage)
    g_flash_map[HAL_FLASH_PART_LOGS].dev    =
        FIXED_PARTITION_DEVICE(logs_storage);
    g_flash_map[HAL_FLASH_PART_LOGS].offset =
        FIXED_PARTITION_OFFSET(logs_storage);
    g_flash_map[HAL_FLASH_PART_LOGS].size   =
        FIXED_PARTITION_SIZE(logs_storage);
    g_flash_map[HAL_FLASH_PART_LOGS].ready  =
        device_is_ready(g_flash_map[HAL_FLASH_PART_LOGS].dev);
#else
    g_flash_map[HAL_FLASH_PART_LOGS].ready = false;
#endif

#if FIXED_PARTITION_EXISTS(calib_storage)
    g_flash_map[HAL_FLASH_PART_CALIB].dev    =
        FIXED_PARTITION_DEVICE(calib_storage);
    g_flash_map[HAL_FLASH_PART_CALIB].offset =
        FIXED_PARTITION_OFFSET(calib_storage);
    g_flash_map[HAL_FLASH_PART_CALIB].size   =
        FIXED_PARTITION_SIZE(calib_storage);
    g_flash_map[HAL_FLASH_PART_CALIB].ready  =
        device_is_ready(g_flash_map[HAL_FLASH_PART_CALIB].dev);
#else
    g_flash_map[HAL_FLASH_PART_CALIB].ready = false;
#endif

    return PROJ_OK;
}

/* ============================================================
 * Internal: get map
 * ============================================================ */
static hal_flash_map_t *get_map(hal_flash_part_t part)
{
    if (part >= HAL_FLASH_PART_COUNT) {
        return NULL;
    }
    return &g_flash_map[part];
}

/* ============================================================
 * READ
 * ============================================================ */
proj_status_t hal_flash_read(hal_flash_part_t part,
                             uint32_t         offset,
                             void            *buf,
                             size_t           len)
{
    VALIDATE_NOT_NULL(buf);
    VALIDATE_RETURN(len == 0U, PROJ_ERR_INVALID);

    hal_flash_map_t *m = get_map(part);
    if (m == NULL || !m->ready || m->dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    if (((size_t)offset + len) > m->size) {
        return PROJ_ERR_OVERFLOW;
    }

    int rc = flash_read(m->dev, m->offset + (off_t)offset, buf, len);
    return (rc == 0) ? PROJ_OK : PROJ_ERR_HW;
}

/* ============================================================
 * WRITE
 * ============================================================ */
proj_status_t hal_flash_write(hal_flash_part_t part,
                              uint32_t         offset,
                              const void      *buf,
                              size_t           len)
{
    VALIDATE_NOT_NULL(buf);
    VALIDATE_RETURN(len == 0U, PROJ_ERR_INVALID);

    hal_flash_map_t *m = get_map(part);
    if (m == NULL || !m->ready || m->dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    if (((size_t)offset + len) > m->size) {
        return PROJ_ERR_OVERFLOW;
    }

    int rc = flash_write(m->dev, m->offset + (off_t)offset, buf, len);
    return (rc == 0) ? PROJ_OK : PROJ_ERR_HW;
}

/* ============================================================
 * ERASE
 * ============================================================ */
proj_status_t hal_flash_erase(hal_flash_part_t part)
{
    hal_flash_map_t *m = get_map(part);
    if (m == NULL || !m->ready || m->dev == NULL) {
        return PROJ_ERR_NOT_INIT;
    }

    int rc = flash_erase(m->dev, m->offset, m->size);
    return (rc == 0) ? PROJ_OK : PROJ_ERR_HW;
}

/* ============================================================
 * GEOMETRY
 * ============================================================ */
uint32_t hal_flash_size(hal_flash_part_t part)
{
    hal_flash_map_t *m = get_map(part);
    if (m == NULL || !m->ready) {
        return 0U;
    }
    return (uint32_t)m->size;
}

uint32_t hal_flash_write_block_size(hal_flash_part_t part)
{
    hal_flash_map_t *m = get_map(part);
    if (m == NULL || !m->ready || m->dev == NULL) {
        return 0U;
    }

    const struct flash_parameters *p = flash_get_parameters(m->dev);
    return (p != NULL) ? (uint32_t)p->write_block_size : 0U;
}

uint32_t hal_flash_erase_block_size(hal_flash_part_t part)
{
    hal_flash_map_t *m = get_map(part);
    if (m == NULL || !m->ready || m->dev == NULL) {
        return 0U;
    }

    struct flash_pages_info info;
    int rc = flash_get_page_info_by_offs(m->dev, m->offset, &info);
    if (rc != 0) {
        return 0U;
    }
    return (uint32_t)info.size;
}

bool hal_flash_is_ready(hal_flash_part_t part)
{
    hal_flash_map_t *m = get_map(part);
    return (m != NULL) && m->ready;
}