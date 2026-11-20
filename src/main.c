/* ============================================================
 * MAIN — ZEPHYR APPLICATION ENTRY
 * ============================================================ */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/fatal.h>

#include <math.h>
#include <string.h>

#include <stm32_ll_spi.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_gpio.h>

#include "config/project_config.h"
#include "config/system_types.h"
#include "config/validation.h"
#include "config/error_handler.h"
#include "hal/hal_time.h"
#include "hal/hal_gpio.h"
#include "hal/hal_spi.h"
#include "hal/hal_uart.h"
#include "hal/hal_cache.h"
#include "hal/hal_ipc.h"
#include "hal/hal_flash.h"
#include "hal/hal_reset.h"
#include "hal/hal_watchdog.h"
#include "hal/hal_dma.h"
#include "hal/hal_power.h"
#include "hal/hal_clock.h"
#include "hal/hal_cpu.h"
#include "common/logger.h"
#include "common/metrics.h"
#include "common/config_loader.h"
#include "common/spsc_queue.h"
#include "ai_icm_virtual.h"
#include "imu_icm42688.h"
#include "cm4_ipc.h"
#include "agent_main_hook.h"
#include "agent.h"
#include "agent_context.h"
#include "agent_ai_if.h"
#include "agent_ekf_if.h"
#include "agent_control_if.h"
#include "ros/micro_ros_app.h"
#include "verified_logic/sensor_layer/sal_core.h"
#include "verified_logic/sensor_layer/sal_types.h"

#include "verified_logic/X-CUBE-AI/App/ai_wrapper.h"

static K_THREAD_STACK_DEFINE(sensor_stack, 4096);
static K_THREAD_STACK_DEFINE(agent_stack,  32768);
static K_THREAD_STACK_DEFINE(ros_stack,    16384);
static K_THREAD_STACK_DEFINE(health_stack, 8192);
static K_THREAD_STACK_DEFINE(lora_init_stack, 4096);

static struct k_thread sensor_tcb;
static struct k_thread agent_tcb;
static struct k_thread ros_tcb;
static struct k_thread health_tcb;
static struct k_thread lora_init_tcb;

#define SENSOR_QUEUE_SIZE           16U
#define SENSOR_LOG_INTERVAL_MS      2000U
#define ROS_PUBLISH_INTERVAL_MS     50U

static void             *sensor_queue_storage[SENSOR_QUEUE_SIZE];
static spsc_queue_t      sensor_queue;
static imu_sample_t      g_latest_sample;
static volatile bool     g_latest_sample_valid = false;

static volatile int         g_lora_rc     = -999;
static volatile bool        g_lora_ready  = false;
static volatile bool        g_spi_ok      = true;

/* ---------- Complementary filter state ---------- */
static volatile float g_qw = 1.0f;
static volatile float g_qx = 0.0f;
static volatile float g_qy = 0.0f;
static volatile float g_qz = 0.0f;

static void complementary_filter_update(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                                        int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                                        float mag_x, float mag_y, float mag_z,
                                        float dt)
{
    const float dps_to_rads = 0.0174532925f;
    const float mg_to_ms2 = 0.004788403f;

    float wx = (float)gx_raw * dps_to_rads;
    float wy = (float)gy_raw * dps_to_rads;
    float wz = (float)gz_raw * dps_to_rads;

    float ax = (float)ax_raw * mg_to_ms2;
    float ay = (float)ay_raw * mg_to_ms2;
    float az = (float)az_raw * mg_to_ms2;

    float qw = g_qw, qx = g_qx, qy = g_qy, qz = g_qz;

    float half_dt = 0.5f * dt;
    float dqx = wx * half_dt;
    float dqy = wy * half_dt;
    float dqz = wz * half_dt;
    float dqw = 1.0f;

    float n = sqrtf(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    if (n > 1e-9f) {
        dqw /= n; dqx /= n; dqy /= n; dqz /= n;

        float new_qw = qw*dqw - qx*dqx - qy*dqy - qz*dqz;
        float new_qx = qw*dqx + qx*dqw + qy*dqz - qz*dqy;
        float new_qy = qw*dqy - qx*dqz + qy*dqw + qz*dqx;
        float new_qz = qw*dqz + qx*dqy - qy*dqx + qz*dqw;

        n = sqrtf(new_qw*new_qw + new_qx*new_qx + new_qy*new_qy + new_qz*new_qz);
        if (n > 1e-9f) {
            qw = new_qw / n; qx = new_qx / n; qy = new_qy / n; qz = new_qz / n;
        }
    }

    float ax_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (ax_norm > 0.5f && ax_norm < 15.0f) {
        float r20 = 2.0f * (qx*qz - qw*qy);
        float r21 = 2.0f * (qy*qz + qw*qx);
        float r22 = 1.0f - 2.0f * (qx*qx + qy*qy);

        const float g = 9.80665f;
        float gx_est = -r20 * g;
        float gy_est = -r21 * g;
        float gz_est = -r22 * g;

        float ex = (ay * gz_est - az * gy_est);
        float ey = (az * gx_est - ax * gz_est);
        float ez = (ax * gy_est - ay * gx_est);

        float exn = sqrtf(ex*ex + ey*ey + ez*ez);
        if (exn > 1e-6f) {
            float gain = 0.02f;
            float dq_x = ex * gain * 0.5f;
            float dq_y = ey * gain * 0.5f;
            float dq_z = ez * gain * 0.5f;
            float dq_w = 1.0f;

            float dn = sqrtf(dq_w*dq_w + dq_x*dq_x + dq_y*dq_y + dq_z*dq_z);
            if (dn > 1e-9f) {
                dq_w /= dn; dq_x /= dn; dq_y /= dn; dq_z /= dn;

                float nqw = qw*dq_w - qx*dq_x - qy*dq_y - qz*dq_z;
                float nqx = qw*dq_x + qx*dq_w + qy*dq_z - qz*dq_y;
                float nqy = qw*dq_y - qx*dq_z + qy*dq_w + qz*dq_x;
                float nqz = qw*dq_z + qx*dq_y - qy*dq_x + qz*dq_w;

                n = sqrtf(nqw*nqw + nqx*nqx + nqy*nqy + nqz*nqz);
                if (n > 1e-9f) {
                    qw = nqw / n; qx = nqx / n; qy = nqy / n; qz = nqz / n;
                }
            }
        }
    }

    float mag_norm = sqrtf(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);
    if (mag_norm > 1.0f && mag_norm < 1000.0f) {
        float r00 = 1.0f - 2.0f * (qy*qy + qz*qz);
        float r01 = 2.0f * (qx*qy - qw*qz);
        float r02 = 2.0f * (qx*qz + qw*qy);
        float r10 = 2.0f * (qx*qy + qw*qz);
        float r11 = 1.0f - 2.0f * (qx*qx + qz*qz);
        float r12 = 2.0f * (qy*qz - qw*qx);

        float mx_w = r00 * mag_x + r01 * mag_y + r02 * mag_z;
        float my_w = r10 * mag_x + r11 * mag_y + r12 * mag_z;

        float heading_meas = atan2f(my_w, mx_w);
        float heading_q = atan2f(2.0f * (qw*qz + qx*qy),
                                 1.0f - 2.0f * (qy*qy + qz*qz));

        float yaw_err = heading_meas - heading_q;
        while (yaw_err > 3.14159f) yaw_err -= 6.28318f;
        while (yaw_err < -3.14159f) yaw_err += 6.28318f;

        float gain_yaw = 0.02f;
        float dq_z = yaw_err * gain_yaw * 0.5f;
        float dq_w = 1.0f;

        float dn = sqrtf(dq_w*dq_w + dq_z*dq_z);
        if (dn > 1e-9f) {
            dq_w /= dn; dq_z /= dn;

            float nqw = qw*dq_w - qz*dq_z;
            float nqx = qx*dq_w + qy*dq_z;
            float nqy = qy*dq_w - qx*dq_z;
            float nqz = qz*dq_w + qw*dq_z;

            n = sqrtf(nqw*nqw + nqx*nqx + nqy*nqy + nqz*nqz);
            if (n > 1e-9f) {
                qw = nqw / n; qx = nqx / n; qy = nqy / n; qz = nqz / n;
            }
        }
    }

    g_qw = qw; g_qx = qx; g_qy = qy; g_qz = qz;
}

void get_quaternion(float *w, float *x, float *y, float *z) {
    *w = g_qw; *x = g_qx; *y = g_qy; *z = g_qz;
}

/* ---------- SPI self-test ---------- */
static K_THREAD_STACK_DEFINE(st_stack, 2048);
static struct k_thread st_tcb;
static K_SEM_DEFINE(st_done, 0, 1);
static volatile char     g_st_test = '-';
static volatile uint32_t g_st_len  = 0U;
static volatile uint32_t g_st_iter = 0U;
static volatile int      g_st_fail = 0;

static void st_entry(void *p1, void *p2, void *p3) {
    uint8_t buf[14]; uint8_t id = 0U;
    g_st_test = 'A'; g_st_len = 2U;
    (void)hal_spi_reg_read(HAL_SPI_BUS_IMU, ICM_REG_WHO_AM_I, &id, 1000U);
    if (id != ICM_WHO_AM_I_VALUE) { g_st_fail++; }
    g_st_test = 'B';
    for (uint32_t n = 1U; n <= 14U; n++) { g_st_len = n + 1U; (void)hal_spi_burst_read(HAL_SPI_BUS_IMU, ICM_REG_ACCEL_DATA_X1, buf, n, 1000U); }
    g_st_test = 'C'; g_st_len = 15U;
    for (uint32_t i = 0U; i < 1000U; i++) { g_st_iter = i; (void)hal_spi_burst_read(HAL_SPI_BUS_IMU, ICM_REG_ACCEL_DATA_X1, buf, 14U, 1000U); k_usleep(1000); }
    g_st_test = 'Z'; k_sem_give(&st_done);
}

static bool spi_selftest(void) {
    k_sem_reset(&st_done); g_st_test = '-'; g_st_len = 0U; g_st_iter = 0U; g_st_fail = 0;
    k_thread_create(&st_tcb, st_stack, K_THREAD_STACK_SIZEOF(st_stack), st_entry, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
    k_thread_name_set(&st_tcb, "spi_test");
    if (k_sem_take(&st_done, K_SECONDS(5)) != 0) {
        printk("\n[SELFTEST] HUNG at '%c', len=%u, iter=%u\n", (char)g_st_test, (unsigned)g_st_len, (unsigned)g_st_iter);
        k_thread_abort(&st_tcb); return false;
    }
    (void)k_thread_join(&st_tcb, K_SECONDS(1)); return true;
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf) {
    printk("\n*** [FATAL] reason=%u\n", reason); k_fatal_halt(reason);
}

static void log_all_sensors(void)
{
    sal_imu_data_t  s_imu;
    sal_mag_data_t  s_mag;
    sal_baro_data_t s_baro;
    sal_lora_data_t s_lora;

    sal_get_imu(&s_imu);
    sal_get_mag(&s_mag);
    sal_get_baro(&s_baro);
    sal_get_lora(&s_lora);

    LOG_I(LOG_TAG_MAIN,
          "[SENS] IMU(%d,%d,%d) MAG(%.1f) BARO(%.0f) LORA(t=%u,r=%u)",
          s_imu.ax, s_imu.ay, s_imu.az,
          (double)s_mag.heading_deg,
          (double)s_baro.pressure_pa,
          (unsigned)s_lora.tx_count, (unsigned)s_lora.rx_count);
}

static void lora_init_thread_entry(void *p1, void *p2, void *p3) {
    k_thread_name_set(k_current_get(), "lora_init");
    k_msleep(500);
    int sal_rc = (int)sal_init_all();

    sal_health_t lora_h = sal_get_health("lora");
    g_lora_ready = (lora_h == SAL_HEALTH_OK);

    LOG_I(LOG_TAG_MAIN, "[LORA-INIT] rc=%d ready=%d",
          sal_rc, (int)g_lora_ready);
}

static void sensor_thread_entry(void *p1, void *p2, void *p3) {
    k_thread_name_set(k_current_get(), "sensor");
    k_msleep(500);
    uint32_t next_wake_us = hal_time_us_32();
    uint32_t last_sensor_log_ms = 0U;
    uint32_t last_lora_tx_ms = 0U;

    while (1) {
        imu_sample_t imu;
        agent_status_t ast = icm_read_sample((agent_sensor_sample_t *)&imu);

        if (ast == AGENT_OK) {
            /* NOTE: Do NOT call agent_health_set_imu() here.
             * agent_thread calls it every step via imu_health().
             * Calling from two threads corrupts imu_fail_accum_us.
             * The ICM driver updates g_icm_health internally. */

            ai_model_feed_sample((const agent_sensor_sample_t *)&imu);
            (void)spsc_push(&sensor_queue, &imu);
            g_latest_sample = imu; g_latest_sample_valid = true;

            {
                sal_mag_data_t mag;
                float mx = 0.0f, my = 0.0f, mz = 0.0f;
                if (sal_get_mag(&mag) == SAL_OK && mag.health == SAL_HEALTH_OK) {
                    mx = mag.mag_ut[0];
                    my = mag.mag_ut[1];
                    mz = mag.mag_ut[2];
                }
                complementary_filter_update(imu.ax, imu.ay, imu.az,
                                            imu.gx, imu.gy, imu.gz,
                                            mx, my, mz,
                                            0.005f);
            }

            (void)sal_read_all();

            uint32_t now_ms_sensors = hal_time_ms_32();
            if ((now_ms_sensors - last_sensor_log_ms) >= SENSOR_LOG_INTERVAL_MS) {
                last_sensor_log_ms = now_ms_sensors;
                log_all_sensors();
            }

            uint32_t now_ms_lora = hal_time_ms_32();
            if (g_lora_ready && (now_ms_lora - last_lora_tx_ms) >= 5000U) {
                last_lora_tx_ms = now_ms_lora;
                extern sal_status_t sal_lora_send_raw(const uint8_t *data, uint16_t len);
                extern volatile bool g_lora_just_received;

                if (g_lora_just_received) {
                    g_lora_just_received = false;
                    g_lora_rc = (int)sal_lora_send_raw((const uint8_t *)"CONFIRM", 7);
                    LOG_I(LOG_TAG_MAIN, "[LORA] CONFIRM rc=%d", g_lora_rc);
                } else {
                    g_lora_rc = (int)sal_lora_send_raw((const uint8_t *)"hello", 5);
                }
            }
        }
        /* else: do NOT call agent_health_set_imu() here either. */

        next_wake_us += CFG_SENSOR_THREAD_PERIOD_US;
        uint32_t now_us = hal_time_us_32();
        if ((int32_t)(next_wake_us - now_us) > 0) k_usleep(next_wake_us - now_us);
        else next_wake_us = now_us;
    }
}

static void agent_thread_entry(void *p1, void *p2, void *p3) {
    k_thread_name_set(k_current_get(), "agent");
    k_msleep(1000);
    uint32_t next_wake_us = hal_time_us_32();
    while (1) {
        void *imu_ptr = NULL;
        while (spsc_pop(&sensor_queue, &imu_ptr) == PROJ_OK) { (void)imu_ptr; }
        (void)agent_main_step();
        next_wake_us += CFG_AGENT_THREAD_PERIOD_US;
        uint32_t now_us = hal_time_us_32();
        if ((int32_t)(next_wake_us - now_us) > 0) k_usleep(next_wake_us - now_us);
        else next_wake_us = now_us;
    }
}

static void ros_thread_entry(void *p1, void *p2, void *p3) {
    k_thread_name_set(k_current_get(), "ros");
    k_msleep(1500);

    if (micro_ros_app_init() != PROJ_OK) {
        LOG_E(LOG_TAG_ROS, "[ROS] init FAILED");
        return;
    }

    uint32_t last_pub_ms = hal_time_ms_32();

    while (1) {
        (void)micro_ros_app_spin(5U);

        uint32_t now = hal_time_ms_32();
        if ((now - last_pub_ms) >= ROS_PUBLISH_INTERVAL_MS) {
            last_pub_ms = now;

            agent_ekf_state_t ekf;
            if (ekf_get_state(&ekf) == AGENT_OK) {
                (void)micro_ros_publish_pose(&ekf);
            }
            if (g_latest_sample_valid) {
                (void)micro_ros_publish_imu((const agent_sensor_sample_t *)&g_latest_sample);
            }
            (void)micro_ros_publish_health();
        }
    }
}

static void health_thread_entry(void *p1, void *p2, void *p3) {
    k_thread_name_set(k_current_get(), "health");
    k_msleep(500);
    while (1) {
        k_msleep(1000);
        (void)hal_watchdog_kick();
    }
}

int main(void) {
    (void)hal_power_init(); (void)hal_clock_init(); (void)hal_cpu_init();
    (void)logger_init(LOG_LEVEL_INFO); (void)error_handler_init(); (void)config_loader_init();
    (void)hal_reset_init(); (void)hal_cache_init(); (void)hal_gpio_init(); (void)hal_spi_init();
    (void)hal_uart_init(); (void)hal_dma_init(); (void)hal_flash_init();
    (void)metrics_init(); (void)spsc_init(&sensor_queue, sensor_queue_storage, SENSOR_QUEUE_SIZE);
    (void)cm4_ipc_init();

    (void)icm_init();

    if (g_spi_ok) g_spi_ok = spi_selftest();

    k_thread_create(&lora_init_tcb, lora_init_stack, K_THREAD_STACK_SIZEOF(lora_init_stack),
                    lora_init_thread_entry, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

    k_msleep(200);

    agent_status_t agent_ret = agent_main_init();
    if (agent_ret != AGENT_OK) {
        LOG_E(LOG_TAG_MAIN, "[BOOT] AGENT INIT FAILED ret=%d", (int)agent_ret);
    }

    if (g_spi_ok) k_thread_create(&sensor_tcb, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack),
                                  sensor_thread_entry, NULL, NULL, NULL, CFG_SENSOR_THREAD_PRIO, 0, K_NO_WAIT);
    k_msleep(200);

    k_thread_create(&agent_tcb, agent_stack, K_THREAD_STACK_SIZEOF(agent_stack),
                    agent_thread_entry, NULL, NULL, NULL, CFG_AGENT_THREAD_PRIO, 0, K_NO_WAIT);
    k_msleep(200);

    k_thread_create(&ros_tcb, ros_stack, K_THREAD_STACK_SIZEOF(ros_stack),
                    ros_thread_entry, NULL, NULL, NULL, CFG_ROS_THREAD_PRIO, 0, K_NO_WAIT);
    k_msleep(200);

    k_thread_create(&health_tcb, health_stack, K_THREAD_STACK_SIZEOF(health_stack),
                    health_thread_entry, NULL, NULL, NULL, 4, 0, K_NO_WAIT);
    k_msleep(200);

    (void)hal_watchdog_init(CFG_WDT_TIMEOUT_US / 1000U);

    LOG_I(LOG_TAG_MAIN, "[BOOT] === ALL SYSTEMS GO ===");
    return 0;
}