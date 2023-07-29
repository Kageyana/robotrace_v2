/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "timer.h"
#include "WS2812C.h"
#include "BMI088.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "switch.h"
#include "motor.h"
#include "battery.h"
#include "encoder.h"
#include "setup.h"
#include "SDcard.h"
#include "markerSensor.h"
#include "lineSensor.h"
#include "PIDcontrol.h"
#include "control.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SIDEMARKER_L_Pin GPIO_PIN_13
#define SIDEMARKER_L_GPIO_Port GPIOC
#define RGBLED_Pin GPIO_PIN_14
#define RGBLED_GPIO_Port GPIOC
#define MOTOR_DIR_L_Pin GPIO_PIN_15
#define MOTOR_DIR_L_GPIO_Port GPIOC
#define MOTOR_CM1_Pin GPIO_PIN_0
#define MOTOR_CM1_GPIO_Port GPIOC
#define BATTM_Pin GPIO_PIN_4
#define BATTM_GPIO_Port GPIOC
#define MOTOR_CM2_Pin GPIO_PIN_5
#define MOTOR_CM2_GPIO_Port GPIOC
#define MOTOR_DIR_R_Pin GPIO_PIN_2
#define MOTOR_DIR_R_GPIO_Port GPIOB
#define SIDEMARKER_R_Pin GPIO_PIN_12
#define SIDEMARKER_R_GPIO_Port GPIOB
#define ButtonR_Pin GPIO_PIN_10
#define ButtonR_GPIO_Port GPIOA
#define ButtonL_Pin GPIO_PIN_11
#define ButtonL_GPIO_Port GPIOA
#define IMU_CSB2_Pin GPIO_PIN_12
#define IMU_CSB2_GPIO_Port GPIOA
#define IMU_CSB1_Pin GPIO_PIN_15
#define IMU_CSB1_GPIO_Port GPIOA
#define CS_MSD_Pin GPIO_PIN_2
#define CS_MSD_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim13;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
