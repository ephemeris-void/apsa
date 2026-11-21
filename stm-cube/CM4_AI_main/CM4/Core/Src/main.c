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

/* ============================================================
 * PERMANENT FIX (ST Community verified — March 2025):
 *   DUAL_CORE_BOOT_SYNC_SEQUENCE is DISABLED.
 *   CM4 STOP mode breaks SWD debug + CM7 flash.
 * ============================================================ */
// #define DUAL_CORE_BOOT_SYNC_SEQUENCE

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
static void MPU_Config(void);
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

  /* Configure MPU — SRAM4 for IPC */
  MPU_Config();

  /* Enable DWT cycle counter for AI latency measurement */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END Init */

  /* Initialize X-CUBE-AI */
  MX_X_CUBE_AI_Init();

  /* USER CODE BEGIN 2 */

  /* ============================================================
   * IPC INIT — wait for CM7 to set magic
   * ============================================================
   * PERMANENT FIX: CM4 runs before CM7 in this config.
   * Wait until CM7 initializes shared memory (magic set).
   * Bounded loop with timeout to avoid infinite hang.
   * ============================================================ */
  uint32_t ipc_timeout = 100000U;
  while (!cm4_ipc_init()) {
      HAL_Delay(10);
      if (--ipc_timeout == 0U) {
          break;    /* proceed anyway; CM7 will init later */
      }
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
      ai_i32 batch = ai_network_run(AI_HANDLE_NULL, NULL, NULL);

      /* Check inference result — batch should be 1 */
      if (batch != 1) {
          g_ai_out.flags = 0xFFFFU;   /* mark inference FAILED */
      } else {
          g_ai_out.flags = 0U;        /* inference OK */
      }

      /* ---------- 5. Prepare output message ---------- */
      g_ai_out.sequence_id  = g_ai_in.sequence_id;
      g_ai_out.timestamp_us = cm4_now_us();

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

/* ============================================================
 * MPU CONFIG — CM4
 * ============================================================
 * Region 0: Peripheral space — non-cacheable
 * Region 1: DTCM (0x20000000) — cacheable
 * Region 2: AXI SRAM (0x24000000) — cacheable
 * Region 3: Flash (0x08000000) — cacheable read-only
 * Region 4: SRAM D2 (0x30000000) — cacheable
 * Region 5: SRAM4 (0x38000000) — IPC shared with CM7
 * ============================================================ */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    /* Region 0: Peripheral space — non-cacheable */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x40000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512MB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Region 1: DTCM (0x20000000) — cacheable */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress      = 0x20000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Region 2: AXI SRAM (0x24000000) — cacheable */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress      = 0x24000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Region 3: Flash (0x08000000) — cacheable read-only */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER3;
    MPU_InitStruct.BaseAddress      = 0x08000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_2MB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Region 4: SRAM D2 (0x30000000) — cacheable */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER4;
    MPU_InitStruct.BaseAddress      = 0x30000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Region 5: SRAM4 (0x38000000) — IPC shared with CM7 */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER5;
    MPU_InitStruct.BaseAddress      = 0x38000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* ============================================================
 * ERROR HANDLER — PERMANENT FIX
 * ============================================================
 * Removed __disable_irq() to keep SWD debug interface alive.
 * ============================================================ */
void Error_Handler(void)
{
  /* Do NOT disable IRQ — keeps SWD alive for debug */
  while (1)
  {
    __NOP();
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */