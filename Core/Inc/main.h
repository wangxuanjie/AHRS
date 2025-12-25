/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#pragma pack(push, 1)  
typedef struct
{
	uint16_t cnt_1Hz;
	uint16_t cnt_5Hz;
	uint16_t cnt_10Hz;
	uint16_t cnt_20Hz;
	uint16_t cnt_50Hz;
	uint16_t cnt_100Hz;
	uint16_t cnt_200Hz;
	uint16_t cnt_500Hz;
}Scheduler;

struct sensor_cali_t
{
    bool gyro;
    bool mag;
    uint8_t acc;
    bool cancel_mag;
};
#pragma pack(pop)

extern struct sensor_cali_t _sensor_need_cali;
extern Scheduler _scheduler;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);

/* Private defines -----------------------------------------------------------*/
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOC
#define ICM20602_FSYNC_Pin GPIO_PIN_0
#define ICM20602_FSYNC_GPIO_Port GPIOA
#define ICM20602_CS_Pin GPIO_PIN_4
#define ICM20602_CS_GPIO_Port GPIOA
#define IST8310_RSTN_Pin GPIO_PIN_12
#define IST8310_RSTN_GPIO_Port GPIOB
#define IST8310_READY_Pin GPIO_PIN_13
#define IST8310_READY_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
