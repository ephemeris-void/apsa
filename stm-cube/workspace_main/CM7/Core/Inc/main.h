/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c — Agent Integration
  * Board: NUCLEO-H755ZI-Q
  * CPU:   Cortex-M7 @ 480 MHz, VOS0, I+D Cache
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Includes ---------- */
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ---------- Exported Handles ---------- */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart3;

/* ---------- Debug globals (updated by imu driver, read by main) ---------- */
extern int16_t dbg_ax;
extern int16_t dbg_ay;
extern int16_t dbg_az;
extern int16_t dbg_gx;
extern int16_t dbg_gy;
extern int16_t dbg_gz;

/* ---------- Exported Functions ---------- */
void Error_Handler(void);

/* ---------- Pin Definitions ---------- */
#define IMU_CS_Pin            GPIO_PIN_14
#define IMU_CS_GPIO_Port      GPIOD

#define USART3_TX_Pin         GPIO_PIN_8
#define USART3_TX_GPIO_Port   GPIOD
#define USART3_RX_Pin         GPIO_PIN_9
#define USART3_RX_GPIO_Port   GPIOD

/* ---------- Cortex-M7 Cache / MPU Access ---------- */
#define DWT_CYCCNT_ADDR       0xE0001004UL
#define DWT_CYCCNT_REG        (*(volatile uint32_t *)DWT_CYCCNT_ADDR)

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */