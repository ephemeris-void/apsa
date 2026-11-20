#ifndef SAL_TYPES_H
#define SAL_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * SENSOR ABSTRACTION LAYER — DATA TYPES
 * ============================================================
 * Board: NUCLEO-H755ZI-Q
 * All sensor data structures — unified across modules.
 * ============================================================ */

/* ---------- Return codes ---------- */
typedef enum {
    SAL_OK          =  0,
    SAL_ERR         = -1,
    SAL_ERR_NOT_INIT= -2,
    SAL_ERR_TIMEOUT = -3,
    SAL_ERR_HW      = -4,
    SAL_ERR_INVALID = -5
} sal_status_t;

/* ---------- Sensor health ---------- */
typedef enum {
    SAL_HEALTH_UNKNOWN  = 0,
    SAL_HEALTH_OK       = 1,
    SAL_HEALTH_DEGRADED = 2,
    SAL_HEALTH_FAIL     = 3
} sal_health_t;

/* ---------- Bus type ---------- */
typedef enum {
    SAL_BUS_NONE = 0,
    SAL_BUS_SPI,
    SAL_BUS_I2C,
    SAL_BUS_UART,
    SAL_BUS_INTERNAL
} sal_bus_t;

/* ---------- GPS data (NEO-M8N) ---------- */
typedef struct {
    double   lat_deg;
    double   lon_deg;
    float    alt_m;
    float    vel_ned[3];        /* m/s */
    float    hdop;
    uint8_t  fix_type;          /* 0=none, 2=2D, 3=3D */
    uint8_t  satellites;
    uint32_t timestamp_us;
    sal_health_t health;
} sal_gps_data_t;

/* ---------- Magnetometer data (HMC5883L) ---------- */
typedef struct {
    float    mag_ut[3];         /* µT — x, y, z */
    float    heading_deg;       /* 0-360 */
    uint32_t timestamp_us;
    sal_health_t health;
} sal_mag_data_t;

/* ---------- Barometer data (DPS310) ---------- */
typedef struct {
    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;
    uint32_t timestamp_us;
    sal_health_t health;
} sal_baro_data_t;

/* ---------- LoRa data (SX1262) ---------- */
typedef struct {
    uint8_t  rx_buffer[256];
    uint16_t rx_len;
    int16_t  rssi_dbm;
    int8_t   snr_db;
    uint32_t rx_timestamp_us;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_fail_count;
    sal_health_t health;
} sal_lora_data_t;

/* ---------- IMU data (existing) ---------- */
typedef struct {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    uint32_t timestamp_us;
    sal_health_t health;
} sal_imu_data_t;

/* ---------- LoRa TX packet ---------- */
typedef struct {
    uint8_t  data[256];
    uint16_t len;
} sal_lora_tx_t;

#ifdef __cplusplus
}
#endif

#endif /* SAL_TYPES_H */