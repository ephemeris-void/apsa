/* ============================================================
 * MICRO-ROS APP — IMPLEMENTATION
 * ============================================================
 * Uses micro-ROS rclc API for embedded ROS 2.
 *
 * Transport: USART6 (PC6=TX, PC7=RX) — FTDI, 115200 baud.
 * Own IRQ-based UART transport (uart_fifo_callback + ring_buf).
 * This is the verified-working path (no dependency on module's
 * microros_transports.c).
 * ============================================================ */

#include "micro_ros_app.h"
#include "logger.h"
#include "error_handler.h"
#include "hal/hal_time.h"
#include "agent_context.h"
#include "agent.h"

/* micro-ROS rclc */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>

/* micro-ROS transports */
#include <uxr/client/transport.h>
#include <rmw_microros/custom_transport.h>

/* ROS 2 messages */
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int32.h>

/* Zephyr */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 * EXTERNAL — gyro-integrated quaternion from main.c
 * ============================================================ */
extern void get_quaternion(float *w, float *x, float *y, float *z);

/* ============================================================
 * CONFIGURATION — topic names relative
 * (namespace "drone" prepended by rclc_node_init_default)
 * ============================================================ */
#define ROS_NAMESPACE           "drone"
#define ROS_NODE_NAME           "virtual_sensor"

#define ROS_TOPIC_POSE          "pose"
#define ROS_TOPIC_IMU           "imu"
#define ROS_TOPIC_HEALTH        "health"
#define ROS_TOPIC_CONTROL       "control"
#define ROS_TOPIC_FAULT         "fault"

#define ROS_QOS_DEPTH           10U
#define ROS_AGENT_PING_TIMEOUT_MS   1000U
#define ROS_AGENT_PING_ATTEMPTS     5U
#define ROS_SPIN_BUDGET_MS          5U

#define ROS_RING_BUF_SIZE           2048

/* ============================================================
 * TRANSPORT — USART6 (FTDI, PC6=TX, PC7=RX)
 * ============================================================ */
static const struct device *ros_uart_dev = NULL;
static bool                 g_transport_open = false;

static char             uart_in_buffer[ROS_RING_BUF_SIZE];
static struct ring_buf  in_ringbuf;

static void uart_fifo_callback(const struct device *dev, void *args)
{
    ARG_UNUSED(args);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            char buffer[64];
            size_t space = ring_buf_space_get(&in_ringbuf);
            size_t len = MIN(space, sizeof(buffer));
            if (len > 0) {
                int recv_len = uart_fifo_read(dev, buffer, len);
                if (recv_len > 0) {
                    ring_buf_put(&in_ringbuf, buffer, recv_len);
                }
            }
        }
    }
}

static bool uart_transport_open(struct uxrCustomTransport *transport)
{
    ARG_UNUSED(transport);

    ros_uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart6));
    if (!device_is_ready(ros_uart_dev)) {
        printk("[ROS-TX] USART6 not ready\n");
        return false;
    }

    ring_buf_init(&in_ringbuf, sizeof(uart_in_buffer), uart_in_buffer);
    uart_irq_callback_set(ros_uart_dev, uart_fifo_callback);
    uart_irq_rx_enable(ros_uart_dev);

    printk("[ROS-TX] USART6 opened (IRQ mode)\n");
    g_transport_open = true;
    return true;
}

static bool uart_transport_close(struct uxrCustomTransport *transport)
{
    ARG_UNUSED(transport);
    if (ros_uart_dev != NULL) uart_irq_rx_disable(ros_uart_dev);
    g_transport_open = false;
    return true;
}

static size_t uart_transport_write(struct uxrCustomTransport *transport,
                                    const uint8_t *buf, size_t len,
                                    uint8_t *errcode)
{
    ARG_UNUSED(transport);
    if (!g_transport_open || ros_uart_dev == NULL) { *errcode = 1; return 0; }
    for (size_t i = 0; i < len; i++) uart_poll_out(ros_uart_dev, buf[i]);
    *errcode = 0;
    return len;
}

static size_t uart_transport_read(struct uxrCustomTransport *transport,
                                   uint8_t *buf, size_t len,
                                   int timeout_ms, uint8_t *errcode)
{
    ARG_UNUSED(transport);
    if (!g_transport_open || ros_uart_dev == NULL) { *errcode = 1; return 0; }

    size_t count = 0;
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (count < len) {
        uart_irq_rx_disable(ros_uart_dev);
        count += ring_buf_get(&in_ringbuf, buf + count, len - count);
        uart_irq_rx_enable(ros_uart_dev);

        if (count >= len) break;
        if (k_uptime_get() >= deadline) break;
        k_usleep(50);
    }

    *errcode = 0;
    return count;
}

/* ============================================================
 * INTERNAL STATE
 * ============================================================ */
static bool g_initialized = false;
static bool g_connected   = false;

static rcl_allocator_t  g_allocator;
static rclc_support_t   g_support;
static rcl_node_t       g_node;
static rclc_executor_t  g_executor;

static rcl_publisher_t  g_pub_pose;
static rcl_publisher_t  g_pub_imu;
static rcl_publisher_t  g_pub_health;
static rcl_publisher_t  g_pub_control;
static rcl_publisher_t  g_pub_fault;

static geometry_msgs__msg__PoseStamped  g_msg_pose;
static sensor_msgs__msg__Imu            g_msg_imu;
static std_msgs__msg__String            g_msg_health;
static geometry_msgs__msg__TwistStamped g_msg_control;
static std_msgs__msg__String            g_msg_fault;

static volatile uint32_t g_publish_count = 0;
static volatile uint32_t g_drop_count    = 0;

/* ============================================================
 * INIT
 * ============================================================ */
proj_status_t micro_ros_app_init(void)
{
    printk("[ROS] init: ENTER\n");
    if (g_initialized) return PROJ_OK;

    /* 1. Register own IRQ-based transport */
    rmw_uros_set_custom_transport(
        true,
        NULL,
        uart_transport_open,
        uart_transport_close,
        uart_transport_write,
        uart_transport_read
    );
    printk("[ROS] init: transport registered\n");

    /* 2. Ping agent */
    printk("[ROS] init: ping agent\n");
    rcl_ret_t rc = RMW_RET_ERROR;
    for (int a = 0; a < ROS_AGENT_PING_ATTEMPTS; a++) {
        rc = rmw_uros_ping_agent(ROS_AGENT_PING_TIMEOUT_MS, 1);
        printk("[ROS] init: ping %d rc=%d\n", a + 1, (int)rc);
        if (rc == RMW_RET_OK) break;
    }
    if (rc != RMW_RET_OK) {
        printk("[ROS] init: agent unreachable\n");
        return PROJ_ERR_TIMEOUT;
    }

    /* 3. Allocator */
    g_allocator = rcl_get_default_allocator();

    /* 4. rclc_support_init */
    rc = rclc_support_init(&g_support, 0, NULL, &g_allocator);
    printk("[ROS] init: support rc=%d\n", (int)rc);
    if (rc != RCL_RET_OK) return PROJ_ERR;

    /* 5. Node init */
    rc = rclc_node_init_default(&g_node, ROS_NODE_NAME, ROS_NAMESPACE, &g_support);
    printk("[ROS] init: node rc=%d\n", (int)rc);
    if (rc != RCL_RET_OK) return PROJ_ERR;

    /* 6. Publishers */
    if (rclc_publisher_init_default(&g_pub_pose, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        ROS_TOPIC_POSE) != RCL_RET_OK) {
        printk("[ROS] init: pose FAILED\n"); return PROJ_ERR;
    }
    printk("[ROS] init: pose OK\n");

    if (rclc_publisher_init_default(&g_pub_imu, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        ROS_TOPIC_IMU) != RCL_RET_OK) {
        printk("[ROS] init: imu FAILED\n"); return PROJ_ERR;
    }
    printk("[ROS] init: imu OK\n");

    if (rclc_publisher_init_default(&g_pub_health, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        ROS_TOPIC_HEALTH) != RCL_RET_OK) {
        printk("[ROS] init: health FAILED\n"); return PROJ_ERR;
    }
    printk("[ROS] init: health OK\n");

    if (rclc_publisher_init_default(&g_pub_control, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        ROS_TOPIC_CONTROL) != RCL_RET_OK) {
        printk("[ROS] init: control FAILED\n"); return PROJ_ERR;
    }
    printk("[ROS] init: control OK\n");

    if (rclc_publisher_init_default(&g_pub_fault, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        ROS_TOPIC_FAULT) != RCL_RET_OK) {
        printk("[ROS] init: fault FAILED\n"); return PROJ_ERR;
    }
    printk("[ROS] init: fault OK\n");

    /* 7. Executor */
    rc = rclc_executor_init(&g_executor, &g_support.context, 4, &g_allocator);
    printk("[ROS] init: executor rc=%d\n", (int)rc);
    if (rc != RCL_RET_OK) return PROJ_ERR;

    /* 8. String buffers */
    static char health_str_buf[128];
    g_msg_health.data.data = health_str_buf;
    g_msg_health.data.size = 0;
    g_msg_health.data.capacity = sizeof(health_str_buf);

    static char fault_str_buf[128];
    g_msg_fault.data.data = fault_str_buf;
    g_msg_fault.data.size = 0;
    g_msg_fault.data.capacity = sizeof(fault_str_buf);

    g_initialized = true;
    g_connected   = true;
    printk("[ROS] init: COMPLETE\n");
    return PROJ_OK;
}

/* ============================================================
 * FINI
 * ============================================================ */
void micro_ros_app_fini(void)
{
    if (!g_initialized) return;
    (void)rcl_publisher_fini(&g_pub_pose, &g_node);
    (void)rcl_publisher_fini(&g_pub_imu, &g_node);
    (void)rcl_publisher_fini(&g_pub_health, &g_node);
    (void)rcl_publisher_fini(&g_pub_control, &g_node);
    (void)rcl_publisher_fini(&g_pub_fault, &g_node);
    (void)rclc_executor_fini(&g_executor);
    (void)rcl_node_fini(&g_node);
    (void)rclc_support_fini(&g_support);
    g_initialized = false;
    g_connected = false;
}

/* ============================================================
 * SPIN
 * ============================================================ */
proj_status_t micro_ros_app_spin(uint32_t timeout_ms)
{
    if (!g_initialized) return PROJ_ERR_NOT_INIT;
    uint32_t budget = (timeout_ms > 0U) ? timeout_ms : ROS_SPIN_BUDGET_MS;
    rcl_ret_t rc = rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(budget));
    if (rc != RCL_RET_OK) { g_connected = false; return PROJ_ERR; }
    return PROJ_OK;
}

/* ============================================================
 * PUBLISH: POSE — uses gyro-integrated quaternion
 * ============================================================ */
proj_status_t micro_ros_publish_pose(const agent_ekf_state_t *ekf)
{
    VALIDATE_NOT_NULL(ekf);
    if (!g_initialized) return PROJ_ERR_NOT_INIT;

    g_msg_pose.header.stamp.sec     = (int32_t)(ekf->timestamp_us / 1000000ULL);
    g_msg_pose.header.stamp.nanosec = (uint32_t)((ekf->timestamp_us % 1000000ULL) * 1000ULL);
    g_msg_pose.header.frame_id.data = "map";
    g_msg_pose.header.frame_id.size = 3;
    g_msg_pose.header.frame_id.capacity = 4;

    g_msg_pose.pose.position.x = (double)ekf->pos[0];
    g_msg_pose.pose.position.y = (double)ekf->pos[1];
    g_msg_pose.pose.position.z = (double)ekf->pos[2];

    /* ✅ Instant orientation from gyro integration */
    float qw, qx, qy, qz;
    get_quaternion(&qw, &qx, &qy, &qz);
    g_msg_pose.pose.orientation.w = (double)qw;
    g_msg_pose.pose.orientation.x = (double)qx;
    g_msg_pose.pose.orientation.y = (double)qy;
    g_msg_pose.pose.orientation.z = (double)qz;

    if (rcl_publish(&g_pub_pose, &g_msg_pose, NULL) == RCL_RET_OK) {
        g_publish_count++; return PROJ_OK;
    }
    g_drop_count++; return PROJ_ERR;
}

/* ============================================================
 * PUBLISH: IMU — uses gyro-integrated quaternion
 * ============================================================ */
proj_status_t micro_ros_publish_imu(const agent_sensor_sample_t *imu)
{
    VALIDATE_NOT_NULL(imu);
    if (!g_initialized) return PROJ_ERR_NOT_INIT;

    g_msg_imu.header.stamp.sec     = (int32_t)(imu->timestamp_us / 1000000U);
    g_msg_imu.header.stamp.nanosec = (uint32_t)((imu->timestamp_us % 1000000U) * 1000U);
    g_msg_imu.header.frame_id.data = "imu_link";
    g_msg_imu.header.frame_id.size = 8;
    g_msg_imu.header.frame_id.capacity = 12;

    const float dps_to_rads = 0.0174532925f;
    g_msg_imu.angular_velocity.x = (double)(imu->gx * dps_to_rads);
    g_msg_imu.angular_velocity.y = (double)(imu->gy * dps_to_rads);
    g_msg_imu.angular_velocity.z = (double)(imu->gz * dps_to_rads);

    const float mg_to_ms2 = 0.004788403f;
    g_msg_imu.linear_acceleration.x = (double)(imu->ax * mg_to_ms2);
    g_msg_imu.linear_acceleration.y = (double)(imu->ay * mg_to_ms2);
    g_msg_imu.linear_acceleration.z = (double)(imu->az * mg_to_ms2);

    /* ✅ Instant orientation from gyro integration */
    float qw, qx, qy, qz;
    get_quaternion(&qw, &qx, &qy, &qz);
    g_msg_imu.orientation.w = (double)qw;
    g_msg_imu.orientation.x = (double)qx;
    g_msg_imu.orientation.y = (double)qy;
    g_msg_imu.orientation.z = (double)qz;

    for (int i = 0; i < 9; i++) {
        g_msg_imu.orientation_covariance[i] = 0.0;
        g_msg_imu.angular_velocity_covariance[i] = 0.0;
        g_msg_imu.linear_acceleration_covariance[i] = 0.0;
    }

    if (rcl_publish(&g_pub_imu, &g_msg_imu, NULL) == RCL_RET_OK) {
        g_publish_count++; return PROJ_OK;
    }
    g_drop_count++; return PROJ_ERR;
}

/* ============================================================
 * PUBLISH: HEALTH
 * ============================================================ */
proj_status_t micro_ros_publish_health(void)
{
    if (!g_initialized) return PROJ_ERR_NOT_INIT;

    agent_context_full_t *ctx = agent_context_get();

    int n = snprintf((char *)g_msg_health.data.data,
                     g_msg_health.data.capacity,
                     "state=%d health=%d imu=%d ai=%d faults=%lu iter=%lu",
                     (int)ctx->current_state,
                     (int)ctx->system_health,
                     (int)ctx->imu_health,
                     (int)ctx->ai_health,
                     (unsigned long)ctx->fault_count,
                     (unsigned long)ctx->iteration_count);

    if (n < 0) n = 0;
    if ((size_t)n > g_msg_health.data.capacity) n = g_msg_health.data.capacity;
    g_msg_health.data.size = (size_t)n;

    if (rcl_publish(&g_pub_health, &g_msg_health, NULL) == RCL_RET_OK) {
        g_publish_count++; return PROJ_OK;
    }
    g_drop_count++; return PROJ_ERR;
}

/* ============================================================
 * PUBLISH: CONTROL
 * ============================================================ */
proj_status_t micro_ros_publish_control(const agent_control_out_t *ctrl)
{
    VALIDATE_NOT_NULL(ctrl);
    if (!g_initialized) return PROJ_ERR_NOT_INIT;

    g_msg_control.header.stamp.sec     = (int32_t)(ctrl->timestamp_us / 1000000U);
    g_msg_control.header.stamp.nanosec = (uint32_t)((ctrl->timestamp_us % 1000000U) * 1000U);
    g_msg_control.header.frame_id.data = "base_link";
    g_msg_control.header.frame_id.size = 9;
    g_msg_control.header.frame_id.capacity = 12;

    g_msg_control.twist.linear.x  = (double)ctrl->thrust;
    g_msg_control.twist.linear.y  = 0.0;
    g_msg_control.twist.linear.z  = 0.0;
    g_msg_control.twist.angular.x = (double)ctrl->roll;
    g_msg_control.twist.angular.y = (double)ctrl->pitch;
    g_msg_control.twist.angular.z = (double)ctrl->yaw;

    if (rcl_publish(&g_pub_control, &g_msg_control, NULL) == RCL_RET_OK) {
        g_publish_count++; return PROJ_OK;
    }
    g_drop_count++; return PROJ_ERR;
}

/* ============================================================
 * QUERY
 * ============================================================ */
bool     micro_ros_is_connected(void) { return g_connected; }
uint32_t micro_ros_publish_count(void) { return g_publish_count; }
uint32_t micro_ros_drop_count(void)    { return g_drop_count; }

/* ============================================================
 * FAULT
 * ============================================================ */
proj_status_t micro_ros_publish_fault(const char *reason)
{
    if (!g_initialized) return PROJ_ERR_NOT_INIT;
    if (reason == NULL) return PROJ_ERR_NULL;

    size_t len = strlen(reason);
    if (len >= g_msg_fault.data.capacity) len = g_msg_fault.data.capacity - 1;
    memcpy(g_msg_fault.data.data, reason, len);
    g_msg_fault.data.size = len;

    if (rcl_publish(&g_pub_fault, &g_msg_fault, NULL) == RCL_RET_OK) {
        g_publish_count++; return PROJ_OK;
    }
    g_drop_count++; return PROJ_ERR;
}