/* ============================================================
 * MICRO-ROS APP — INTERFACE
 * ============================================================
 * - micro-ROS node for drone virtual sensor
 * - Publishers: pose, imu, health, control
 * - Subscribers: control_command (optional)
 * - Serial transport over UART2
 * ============================================================ */

#ifndef MICRO_ROS_APP_H
#define MICRO_ROS_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "config/validation.h"
#include "config/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Init / Spin
 * ============================================================ */
proj_status_t micro_ros_app_init(void);
proj_status_t micro_ros_app_spin(uint32_t timeout_ms);
void          micro_ros_app_fini(void);

/* ============================================================
 * Publishers
 * ============================================================ */
proj_status_t micro_ros_publish_pose(const ekf_state_t *ekf);
proj_status_t micro_ros_publish_imu(const imu_sample_t *imu);
proj_status_t micro_ros_publish_health(void);
proj_status_t micro_ros_publish_control(const control_out_t *ctrl);

/* ============================================================
 * Status
 * ============================================================ */
bool     micro_ros_is_connected(void);
uint32_t micro_ros_publish_count(void);
uint32_t micro_ros_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* MICRO_ROS_APP_H */