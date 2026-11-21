/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — CM4 AI Worker
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"
#include "cm4_ipc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "network.h"
#include "network_data.h"
#include "core_cm4.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif
#endif

#define CM4_HEARTBEAT_PERIOD_MS         100U

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static cm4_ipc_ai_msg_t g_ai_in;
static cm4_ipc_ai_msg_t g_ai_out;
static uint32_t         g_inf_count  = 0U;
static uint32_t         g_last_hb_ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* X-CUBE-AI standard API — no custom accessors on CM4 */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static inline uint32_t cm4_now_us(void)
{
    return DWT->CYCCNT / 240U;
}

/* USER CODE END 0 */

int main(void)
{
/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
#endif
/* USER CODE END Boot_Mode_Sequence_1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* Enable DWT cycle counter for AI latency measurement */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END Init */

  /* Initialize X-CUBE-AI */
  MX_X_CUBE_AI_Init();

  /* USER CODE BEGIN 2 */

  /* Initialize IPC — CM4 waits for CM7 to init shared memory */
  if (!cm4_ipc_init()) {
      Error_Handler();
  }

  /* USER CODE END 2 */

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* ---------- 1. Poll CM7 for new AI input ---------- */
    if (cm4_ipc_recv_input(&g_ai_in))
    {
      /* ---------- 2. Get X-CUBE-AI input/output buffers ---------- */
      ai_buffer *ai_in_struct  = ai_network_inputs_get(AI_HANDLE_NULL, NULL);
      ai_buffer *ai_out_struct = ai_network_outputs_get(AI_HANDLE_NULL, NULL);

      int8_t *ai_in_buf  = (int8_t *)ai_in_struct[0].data;
      int8_t *ai_out_buf = (int8_t *)ai_out_struct[0].data;

      /* ---------- 3. Copy IPC input into X-CUBE-AI input buffer ---------- */
      for (uint32_t i = 0U; i < CM4_IPC_AI_BYTES; i++) {
          ai_in_buf[i] = g_ai_in.data[i];
      }

      /* NOTE: CM4 (Cortex-M4) has no D-Cache.
       *       Cache coherency handled by CM7 after IPC transfer.
       */

      /* ---------- 4. Run AI inference ---------- */
      MX_X_CUBE_AI_Process();

      /* ---------- 5. Prepare output message ---------- */
      g_ai_out.sequence_id  = g_ai_in.sequence_id;
      g_ai_out.timestamp_us = cm4_now_us();
      g_ai_out.flags        = 0U;

      for (uint32_t i = 0U; i < CM4_IPC_AI_BYTES; i++) {
          g_ai_out.data[i] = ai_out_buf[i];
      }

      /* ---------- 6. Send result back to CM7 ---------- */
      (void)cm4_ipc_send_output(&g_ai_out);

      g_inf_count++;
    }

    /* ---------- 7. Heartbeat / status update ---------- */
    if ((now - g_last_hb_ms) >= CM4_HEARTBEAT_PERIOD_MS) {
        g_last_hb_ms = now;
        (void)cm4_ipc_status_update();
    }
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */