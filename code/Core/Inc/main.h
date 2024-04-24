/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define PTT_IN_Pin LL_GPIO_PIN_7
#define PTT_IN_GPIO_Port GPIOB
#define PTT_OUT_Pin LL_GPIO_PIN_15
#define PTT_OUT_GPIO_Port GPIOC
#define BAND1_Pin LL_GPIO_PIN_0
#define BAND1_GPIO_Port GPIOA
#define BAND2_Pin LL_GPIO_PIN_1
#define BAND2_GPIO_Port GPIOA
#define BAND3_Pin LL_GPIO_PIN_2
#define BAND3_GPIO_Port GPIOA
#define BAND4_Pin LL_GPIO_PIN_3
#define BAND4_GPIO_Port GPIOA
#define BAND5_Pin LL_GPIO_PIN_4
#define BAND5_GPIO_Port GPIOA
#define BAND6_Pin LL_GPIO_PIN_5
#define BAND6_GPIO_Port GPIOA
#define BAND7_Pin LL_GPIO_PIN_7
#define BAND7_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_0
#define LED_GPIO_Port GPIOB
#define BAND8_Pin LL_GPIO_PIN_11
#define BAND8_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
