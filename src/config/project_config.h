/* ============================================================
 * PROJECT CONFIG — SINGLE SOURCE OF TRUTH (FINAL v7)
 * ============================================================
 * Changes:
 *   - Sensor thread priority 5 (HIGHER than agent)
 *   - Agent thread priority 10 (LOWER than sensor)
 *   - Watchdog timeout 30s (debug-friendly)
 * ============================================================ */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/* ---------- Build Identity ---------- */
#define CFG_PROJECT_NAME        "STM32_AI_Virtual_Sensor"
#define CFG_VERSION_MAJOR       2
#define CFG_VERSION_MINOR       0
#define CFG_VERSION_PATCH       0
#define CFG_BUILD_DATE          __DATE__
#define CFG_BUILD_TIME          __TIME__

/* ---------- CPU / Clock ---------- */
#define CFG_CPU_FREQ_HZ         480000000U
#define CFG_SYSCLK_FREQ_HZ      480000000U
#define CFG_HCLK_FREQ_HZ        240000000U
#define CFG_CYCLES_PER_US       (CFG_CPU_FREQ_HZ / 1000000U)

/* ---------- RTOS / Threads ----------
 * FIX v7: Sensor PRIO=5 (higher), Agent PRIO=10 (lower)
 *         This prevents Agent thread from preempting Sensor thread
 *         during SPI transactions.
 */
#define CFG_SENSOR_THREAD_PRIO      5    /* HIGHER Priority */
#define CFG_AGENT_THREAD_PRIO       10   /* LOWER Priority */
#define CFG_ROS_THREAD_PRIO         7
#define CFG_SENSOR_THREAD_STACK     8192
#define CFG_AGENT_THREAD_STACK      32768
#define CFG_ROS_THREAD_STACK        8192
#define CFG_AGENT_THREAD_PERIOD_US  1000U
#define CFG_SENSOR_THREAD_PERIOD_US 1000U
#define CFG_ROS_PUBLISH_PERIOD_MS   20U

/* ---------- Control Loop Timing ---------- */
#define CFG_LOOP_DEADLINE_US        1000U
#define CFG_LOOP_WCET_BUDGET_US     800U
#define CFG_STATE_MIN_DWELL_US      10000U
#define CFG_STATE_TIMEOUT_US        50000U
#define CFG_EMERGENCY_TIMEOUT_US    300000000U
#define CFG_LANDING_MIN_DWELL_US    1000000U

/* ---------- Sensor Config ---------- */
#define CFG_IMU_ODR_HZ              1000U
#define CFG_IMU_ACCEL_RANGE_G       16
#define CFG_IMU_GYRO_RANGE_DPS      2000
#define CFG_IMU_SPI_FREQ_HZ         3750000U
#define CFG_IMU_WHO_AM_I_EXPECTED   0x47U
#define CFG_IMU_DROPOUT_THRESHOLD_US 1500U
#define CFG_IMU_FAIL_CONFIRM_COUNT  3U
#define CFG_IMU_RECOVER_CONFIRM_COUNT 10U

/* ---------- AI Config ---------- */
#define CFG_AI_INPUT_SAMPLES        512U
#define CFG_AI_INPUT_CHANNELS       4U
#define CFG_AI_INPUT_BYTES          (CFG_AI_INPUT_CHANNELS * CFG_AI_INPUT_SAMPLES)
#define CFG_AI_MAX_LATENCY_US       11000U
#define CFG_AI_TARGET_LATENCY_US    5000U
#define CFG_AI_CONFIDENCE_MIN       0.80f
#define CFG_AI_TRIGGER_PERIOD_MS    20U

/* ---------- EKF / Estimation ---------- */
#define CFG_EKF_STATE_DIM           19U
#define CFG_EKF_ZUPT_ACCEL_TOL      1.0f
#define CFG_EKF_ZUPT_GYRO_TOL       0.30f
#define CFG_EKF_ZUPT_MIN_SAMPLES    50U
#define CFG_EKF_ACCEL_DECIMATION    10U

/* ---------- Health / Fault ---------- */
#define CFG_HEALTH_FAIL_CONFIRM     3U
#define CFG_HEALTH_RECOVER_CONFIRM  10U
#define CFG_FAULT_LOG_DEPTH         128U
#define CFG_FAULT_ESCALATE_THRESH   5U

/* ---------- Watchdog ---------- */
#define CFG_WDT_TIMEOUT_US          30000000U   /* 30 sec for debug */
#define CFG_WDT_KICK_INTERVAL_US    500U
#define CFG_WDT_ENABLE              1

/* ---------- Communication ---------- */
#define CFG_UART_BAUD_RATE          921600U
#define CFG_UART_TX_BUFFER_SIZE     4096U
#define CFG_UART_RX_BUFFER_SIZE     1024U
#define CFG_ROS_TOPIC_IMU           "/drone/imu"
#define CFG_ROS_TOPIC_POSE          "/drone/pose"
#define CFG_ROS_TOPIC_HEALTH        "/drone/health"
#define CFG_ROS_TOPIC_CONTROL       "/drone/control"

/* ---------- IPC (CM7 ↔ CM4) ---------- */
#define CFG_IPC_SHARED_ADDR         0x38000000UL
#define CFG_IPC_RING_SIZE           8U
#define CFG_IPC_TIMEOUT_US          1000U

/* ---------- Envelope (Safety) ---------- */
#define CFG_ENVELOPE_ALT_MAX_M      5000.0f
#define CFG_ENVELOPE_VEL_MAX_MS     200.0f
#define CFG_ENVELOPE_ACCEL_G        16.0f
#define CFG_ENVELOPE_GYRO_DPS       2000.0f

/* ---------- Feature Flags (compile-time) ---------- */
#define CFG_FEATURE_AI              1
#define CFG_FEATURE_ESKF            1
#define CFG_FEATURE_ROS             1
#define CFG_FEATURE_IPC             1
#define CFG_FEATURE_WATCHDOG        1
#define CFG_FEATURE_TRACE           1
#define CFG_FEATURE_METRICS         1
#define CFG_FEATURE_DCACHE          1

/* ---------- Build-time Validation ---------- */
#if CFG_CPU_FREQ_HZ == 0
#error "CFG_CPU_FREQ_HZ must be non-zero"
#endif

#if CFG_LOOP_WCET_BUDGET_US >= CFG_LOOP_DEADLINE_US
#error "WCET budget must be less than deadline"
#endif

#if CFG_AI_INPUT_SAMPLES == 0
#error "AI input samples must be > 0"
#endif

#if CFG_AGENT_THREAD_STACK < 32768
#error "Agent stack too small (min 32KB)"
#endif

#if CFG_SENSOR_THREAD_STACK < 8192
#error "Sensor stack too small (min 8KB)"
#endif

#if CFG_WDT_TIMEOUT_US < 1000000U
#error "WDT timeout too small (min 1 sec)"
#endif

#if CFG_FEATURE_ROS && !CFG_FEATURE_IPC
#warning "ROS without IPC may limit communication"
#endif

/* ---------- Stringify helpers ---------- */
#define STR(x)  #x
#define XSTR(x) STR(x)

/* ---------- Build Identity String ---------- */
#define CFG_VERSION_STRING  \
    CFG_PROJECT_NAME " v"   \
    XSTR(CFG_VERSION_MAJOR) "."  \
    XSTR(CFG_VERSION_MINOR) "."  \
    XSTR(CFG_VERSION_PATCH)

#endif /* PROJECT_CONFIG_H */