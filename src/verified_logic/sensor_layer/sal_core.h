#ifndef SAL_CORE_H
#define SAL_CORE_H

#include "sal_types.h"
#include "sal_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * SENSOR ABSTRACTION LAYER — CORE API
 * ============================================================ */

typedef struct {
    const char   *name;
    sal_bus_t     bus;
    uint32_t      rate_hz;
    uint32_t      timeout_ms;

    /* Function pointers */
    sal_status_t (*init)(void);
    sal_status_t (*read)(void *out);
    sal_status_t (*health_fn)(void);     /* ← renamed: was `health` */
    void         (*deinit)(void);

    /* Internal state */
    sal_health_t  health;
    uint32_t      last_update_us;
    uint32_t      read_count;
    uint32_t      fail_count;
    bool          registered;
} sal_sensor_t;

sal_status_t sal_register(const sal_sensor_t *sensor);

sal_status_t sal_init_all(void);
sal_status_t sal_read_all(void);
sal_status_t sal_health_all(void);
void         sal_deinit_all(void);

sal_status_t sal_get_imu(sal_imu_data_t   *out);
sal_status_t sal_get_gps(sal_gps_data_t   *out);
sal_status_t sal_get_mag(sal_mag_data_t   *out);
sal_status_t sal_get_baro(sal_baro_data_t *out);
sal_status_t sal_get_lora(sal_lora_data_t *out);

sal_health_t sal_get_health(const char *name);

sal_status_t sal_lora_send(const uint8_t *data, uint16_t len);

typedef struct {
    uint32_t registered_count;
    uint32_t init_ok_count;
    uint32_t init_fail_count;
    uint32_t read_ok_count;
    uint32_t read_fail_count;
} sal_stats_t;

const sal_stats_t* sal_get_stats(void);

void sal_dump_registry(void);

#ifdef __cplusplus
}
#endif

#endif /* SAL_CORE_H */