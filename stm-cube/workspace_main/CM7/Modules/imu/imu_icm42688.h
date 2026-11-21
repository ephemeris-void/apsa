#ifndef IMU_ICM42688_H
#define IMU_ICM42688_H

#include "../agent/core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * ICM-42688-P IMU DRIVER
 * ============================================================
 * Board:  NUCLEO-H755ZI-Q
 * Bus:    SPI1 @ 3.84 MHz
 * Pins:   PA5=SCK, PA6=MISO, PB5=MOSI, PD14=CS
 * ODR:    1 kHz
 * Range:  ±16g, ±2000 dps
 *
 * Runtime fail detection:
 *   - Range violation   → DEGRADED
 *   - All-zero pattern  → FAIL
 *   - 10 consecutive OK → OK (recovery)
 * ============================================================ */

/* ---------- Register Map ---------- */
#define ICM_REG_WHO_AM_I        0x75
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_ACCEL_DATA_X1   0x1F
#define ICM_REG_TEMP_DATA1      0x1D

/* ---------- Expected Values ---------- */
#define ICM_WHO_AM_I_VALUE      0x47
#define ICM_PWR_MGMT0_LN        0x0F    /* ACCEL LN + GYRO LN */
#define ICM_ODR_1KHZ            0x06    /* 1kHz, ±16g / ±2000dps */

/* ---------- Driver API ---------- */
agent_status_t  icm_init(void);
agent_status_t  icm_read_sample(agent_sensor_sample_t *sample);
agent_health_t  icm_health(void);
void            icm_on_failure(void);
void            icm_set_health_ok(void);   /* ← force OK for main.c init */

/* ---------- Low-Level ---------- */
uint8_t         icm_read_reg(uint8_t reg);
void            icm_write_reg(uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ICM42688_H */