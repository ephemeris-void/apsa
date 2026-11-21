#ifndef AGENT_IMU_IF_H
#define AGENT_IMU_IF_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * IMU INTERFACE — FROZEN
 * ============================================================
 * External module: IMU Driver (Modules/imu/)
 * Link point:      imu_icm42688.c / imu_icm42688.h
 * Hardware:        ICM-42688-P @ SPI1
 * Board:           NUCLEO-H755ZI-Q
 *
 * To replace IMU (e.g., BMI088, ASM330LHH):
 *   1. Implement these functions in new driver
 *   2. This interface stays UNCHANGED
 *   3. Agent code stays UNCHANGED
 *
 * WCET budget:
 *   imu_init        : 5 ms (boot only)
 *   imu_get_sample  : 50 µs
 *   imu_health      : 1 µs
 * ============================================================ */

/* ---------- Initialization ---------- */
agent_status_t  imu_init(void);

/* ---------- Read Sample (blocking, bounded) ----------
 * Returns AGENT_OK on success, fills sample.
 * If sample == NULL, only health check performed.
 */
agent_status_t  imu_get_sample(agent_sensor_sample_t *sample);

/* ---------- Health ---------- */
agent_health_t  imu_health(void);

/* ---------- Failure Detection (called by driver on fault) ---------- */
void            imu_on_failure(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_IMU_IF_H */