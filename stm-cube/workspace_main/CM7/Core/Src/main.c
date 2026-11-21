/* USER CODE BEGIN Header */
#include "main.h"
#include "app_x-cube-ai.h"
#include "network.h"
#include "network_data.h"
#include "agent_main_hook.h"
#include "agent.h"
#include "agent_ai_if.h"
#include "agent_imu_if.h"
#include "agent_ekf_if.h"
#include "agent_control_if.h"
#include "ai_icm_virtual.h"
#include "cm4_ipc.h"
#include "apsa_bridge/apsa_config.h"
#include "imu_icm42688.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Header */

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;

extern void* AI_Get_Input_Buffer(void);
extern void* AI_Get_Output_Buffer(void);

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);

#ifndef HSEM_ID_0
#define HSEM_ID_0    (0U)
#endif

int16_t dbg_ax = 0;
int16_t dbg_ay = 0;
int16_t dbg_az = 0;
int16_t dbg_gx = 0;
int16_t dbg_gy = 0;
int16_t dbg_gz = 0;

#define DEBUG_UART_TX(msg, len)   HAL_UART_Transmit(&huart3, (uint8_t*)(msg), (len), 100)

static uint32_t dbg_read_bcm4(void)
{
    volatile uint32_t *bcm4r = (volatile uint32_t *)0x52002048UL;
    return *bcm4r;
}

static void dbg_verify_bcm4(void)
{
    char buf[80];
    int len;
    uint32_t bcm4 = dbg_read_bcm4();
    len = sprintf(buf, "DBG_BCM4: 0x%08lX\r\n", (unsigned long)bcm4);
    DEBUG_UART_TX(buf, len);
}

static void dbg_imu_direct_test(void)
{
    char buf[128];
    int len;

    uint8_t who = icm_read_reg(0x75);
    len = sprintf(buf, "DIRECT WHO_AM_I: 0x%02X (expect 0x47)\r\n", who);
    DEBUG_UART_TX(buf, len);

    uint8_t pwr = icm_read_reg(0x4E);
    len = sprintf(buf, "DIRECT PWR_MGMT0: 0x%02X (expect 0x0F)\r\n", pwr);
    DEBUG_UART_TX(buf, len);

    uint8_t acc_cfg = icm_read_reg(0x50);
    len = sprintf(buf, "DIRECT ACCEL_CFG0: 0x%02X (expect 0x06)\r\n", acc_cfg);
    DEBUG_UART_TX(buf, len);

    uint8_t gyr_cfg = icm_read_reg(0x4F);
    len = sprintf(buf, "DIRECT GYRO_CFG0: 0x%02X (expect 0x06)\r\n", gyr_cfg);
    DEBUG_UART_TX(buf, len);

    uint8_t ax_h = icm_read_reg(0x1F);
    uint8_t ax_l = icm_read_reg(0x20);
    uint8_t ay_h = icm_read_reg(0x21);
    uint8_t ay_l = icm_read_reg(0x22);
    uint8_t az_h = icm_read_reg(0x23);
    uint8_t az_l = icm_read_reg(0x24);
    uint8_t gx_h = icm_read_reg(0x25);
    uint8_t gx_l = icm_read_reg(0x26);

    len = sprintf(buf, "RAW: ax=%02X%02X ay=%02X%02X az=%02X%02X gx=%02X%02X\r\n",
                  ax_h, ax_l, ay_h, ay_l, az_h, az_l, gx_h, gx_l);
    DEBUG_UART_TX(buf, len);

    int16_t ax_test = (int16_t)(((uint16_t)ax_h << 8) | ax_l);
    int16_t ay_test = (int16_t)(((uint16_t)ay_h << 8) | ay_l);
    int16_t az_test = (int16_t)(((uint16_t)az_h << 8) | az_l);
    int16_t gx_test = (int16_t)(((uint16_t)gx_h << 8) | gx_l);

    len = sprintf(buf, "INT: ax=%d ay=%d az=%d gx=%d\r\n",
                  ax_test, ay_test, az_test, gx_test);
    DEBUG_UART_TX(buf, len);
}

static void dbg_print_cm4_status(void)
{
    cm4_ipc_status_t st;
    if (cm4_ipc_status_read(&st)) {
        char buf[96];
        int len = sprintf(buf, "CM4: alive=%lu inf=%lu hb=%lu magic=0x%08lX\r\n",
                          (unsigned long)st.cm4_alive,
                          (unsigned long)st.cm4_ai_inference_count,
                          (unsigned long)st.cm4_last_heartbeat_us,
                          (unsigned long)cm4_ipc_get_magic());
        DEBUG_UART_TX(buf, len);
    }
}

int main(void)
{
  MPU_Config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();

  char msg[256];
  int len;

  __HAL_RCC_HSEM_CLK_ENABLE();
  (*(volatile uint32_t *)0x58024538UL) |= (1U << 8);
  HAL_HSEM_Release(HSEM_ID_0, 0U);

  DEBUG_UART_TX("DBG_START\r\n", 11);
  dbg_verify_bcm4();

  if (!cm4_ipc_init()) {
      HAL_UART_Transmit(&huart3, (uint8_t*)"IPC INIT FAIL\r\n", 15, 100);
      Error_Handler();
  }
  HAL_UART_Transmit(&huart3, (uint8_t*)"IPC Initialized\r\n", 17, 100);
  HAL_Delay(50);

  apsa_config_init();
  if (!apsa_config_validate()) {
      HAL_UART_Transmit(&huart3, (uint8_t*)"APSA CFG FAIL\r\n", 15, 100);
      Error_Handler();
  }
  HAL_UART_Transmit(&huart3, (uint8_t*)"APSA Config OK\r\n", 16, 100);
  HAL_Delay(50);

  if (agent_main_init() != AGENT_OK) {
      HAL_UART_Transmit(&huart3, (uint8_t*)"AGENT INIT FAIL\r\n", 17, 100);
      Error_Handler();
  }
  HAL_UART_Transmit(&huart3, (uint8_t*)"Agent Initialized\r\n", 19, 100);
  HAL_Delay(50);

  HAL_UART_Transmit(&huart3, (uint8_t*)"=== IMU DEBUG START ===\r\n", 25, 100);
  dbg_imu_direct_test();
  HAL_UART_Transmit(&huart3, (uint8_t*)"=== IMU DEBUG END ===\r\n", 23, 100);
  HAL_Delay(100);

  /* Set ESKF initial attitude from averaged IMU samples */
  {
      agent_sensor_sample_t imu_init_sample;
      int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
      int valid_samples = 0;

      for (int i = 0; i < 20; i++) {
          if (imu_get_sample(&imu_init_sample) == AGENT_OK) {
              ax_sum += imu_init_sample.ax;
              ay_sum += imu_init_sample.ay;
              az_sum += imu_init_sample.az;
              valid_samples++;
          }
          HAL_Delay(2);
      }

      if (valid_samples > 0) {
          float ax_avg = (float)ax_sum / (float)valid_samples;
          float ay_avg = (float)ay_sum / (float)valid_samples;
          float az_avg = (float)az_sum / (float)valid_samples;

          float ax_ms2 = ax_avg * (9.80665f / 2048.0f);
          float ay_ms2 = ay_avg * (9.80665f / 2048.0f);
          float az_ms2 = az_avg * (9.80665f / 2048.0f);

          ekf_set_initial_attitude_from_accel(ax_ms2, ay_ms2, az_ms2);

          len = sprintf(msg, "ESKF Init Attitude SET: ax=%.4f ay=%.4f az=%.4f m/s2\r\n",
                        ax_ms2, ay_ms2, az_ms2);
          HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
      } else {
          HAL_UART_Transmit(&huart3, (uint8_t*)"ESKF Init Attitude: NO IMU DATA\r\n", 33, 100);
      }
  }
  HAL_Delay(50);

  if (ai_init() != AGENT_OK) {
      HAL_UART_Transmit(&huart3, (uint8_t*)"AI_INIT: FAIL\r\n", 15, 100);
  } else {
      HAL_UART_Transmit(&huart3, (uint8_t*)"AI_INIT: OK\r\n", 13, 100);
  }
  HAL_Delay(50);

  uint32_t last_agent      = 0;
  uint32_t last_status     = 0;
  uint32_t last_cm4_status = 0;
  uint32_t last_ai_trigger = 0;
  uint32_t last_imu_read   = 0;
  uint32_t last_pipe       = 0;

  while(1)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_imu_read) >= 1) {
      last_imu_read = now;
      agent_sensor_sample_t s;
      if (imu_get_sample(&s) == AGENT_OK) {
        dbg_ax = s.ax;
        dbg_ay = s.ay;
        dbg_az = s.az;
        dbg_gx = s.gx;
        dbg_gy = s.gy;
        dbg_gz = s.gz;

        ai_model_feed_sample(&s);
      }
    }

    if ((now - last_agent) >= 1) {
      last_agent = now;
      agent_main_step();
    }

    if ((now - last_ai_trigger) >= 20) {
      last_ai_trigger = now;
      if (ai_status() == AI_STATUS_IDLE) {
          ai_trigger();
      }
    }

    /* ============================================================
     * STRONG PIPELINE — Framed data @ 50 Hz for ROS 2 bridge
     * ============================================================
     * Format: P:..|Q:..|V:..|T:..|t:..|h:..|f:..
     */
    if ((now - last_pipe) >= 20) {
      last_pipe = now;

      agent_ekf_state_t ekf_pipe;
      agent_control_out_t ctrl_pipe;

      if (ekf_get_state(&ekf_pipe) == AGENT_OK &&
          control_get_output(&ctrl_pipe) == AGENT_OK) {

          len = sprintf(msg,
            "P:%.4f,%.4f,%.4f|Q:%.5f,%.5f,%.5f,%.5f|V:%.4f,%.4f,%.4f|T:%.2f|t:%lu|h:%d|f:%lu\n",
            ekf_pipe.pos[0], ekf_pipe.pos[1], ekf_pipe.pos[2],
            ekf_pipe.quat[0], ekf_pipe.quat[1], ekf_pipe.quat[2], ekf_pipe.quat[3],
            ekf_pipe.vel[0], ekf_pipe.vel[1], ekf_pipe.vel[2],
            ctrl_pipe.thrust,
            (unsigned long)now,
            (int)agent_get_health(),
            (unsigned long)agent_get_fault_count());
          HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
      }
    }

    if ((now - last_status) >= 1000) {
      last_status = now;

      len = sprintf(msg,
        "AGENT: state=%s health=%d conf=%d%% faults=%lu iter=%lu wcet=%luus tis=%luus\r\n",
        agent_get_state_name(agent_get_state()),
        (int)agent_get_health(),
        (int)(agent_get_confidence() * 100.0f),
        (unsigned long)agent_get_fault_count(),
        (unsigned long)agent_get_iteration_count(),
        (unsigned long)(agent_get_wcet_max_us() % 100000),
        (unsigned long)agent_time_in_state_us());
      HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);

      len = sprintf(msg, "AI: status=%d ready=%d\r\n",
        (int)ai_status(), (int)ai_output_ready());
      HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);

      len = sprintf(msg, "IMU: ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d\r\n",
        (int)dbg_ax, (int)dbg_ay, (int)dbg_az,
        (int)dbg_gx, (int)dbg_gy, (int)dbg_gz);
      HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);

      agent_ekf_state_t ekf_out;
      if (ekf_get_state(&ekf_out) == AGENT_OK) {
          len = sprintf(msg,
            "ESKF: p=%.2f,%.2f,%.2f v=%.2f,%.2f,%.2f q=%.3f,%.3f,%.3f,%.3f c=%.2f\r\n",
            ekf_out.pos[0], ekf_out.pos[1], ekf_out.pos[2],
            ekf_out.vel[0], ekf_out.vel[1], ekf_out.vel[2],
            ekf_out.quat[0], ekf_out.quat[1], ekf_out.quat[2], ekf_out.quat[3],
            ekf_out.confidence);
          HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
      } else {
          HAL_UART_Transmit(&huart3, (uint8_t*)"ESKF: NOT READY\r\n", 18, 100);
      }

      agent_control_out_t mpc_out;
      if (control_get_output(&mpc_out) == AGENT_OK) {
          len = sprintf(msg,
            "MPC: T=%.2f R=%.2f P=%.2f Y=%.2f\r\n",
            mpc_out.thrust, mpc_out.roll, mpc_out.pitch, mpc_out.yaw);
          HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
      } else {
          HAL_UART_Transmit(&huart3, (uint8_t*)"MPC: NOT READY\r\n", 17, 100);
      }
    }

    if ((now - last_cm4_status) >= 500) {
      last_cm4_status = now;
      dbg_print_cm4_status();
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  uint32_t vos_timeout = 1000000U;
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    if (--vos_timeout == 0U) {
      __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
      break;
    }
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x40000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x20000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER3;
    MPU_InitStruct.BaseAddress = 0x08000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER4;
    MPU_InitStruct.BaseAddress = 0x30000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER5;
    MPU_InitStruct.BaseAddress = 0x38000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
  SPI1->CR1 |= SPI_CR1_SPE;
}

static void MX_USART3_UART_Init(void)
{
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  while(1) {
    __NOP();
  }
}