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
#include "stm32f7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define MS1_SYRINGE_Pin GPIO_PIN_5
#define MS1_SYRINGE_GPIO_Port GPIOA
#define MS2_SYRINGE_Pin GPIO_PIN_6
#define MS2_SYRINGE_GPIO_Port GPIOA
#define MS3_SYRINGE_Pin GPIO_PIN_7
#define MS3_SYRINGE_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DIR_NEEDLE_Pin GPIO_PIN_7
#define DIR_NEEDLE_GPIO_Port GPIOE
#define END_STOP_NEEDLE_1_Pin GPIO_PIN_8
#define END_STOP_NEEDLE_1_GPIO_Port GPIOE
#define END_STOP_NEEDLE_1_EXTI_IRQn EXTI9_5_IRQn
#define END_STOP_SYRINGE_1_Pin GPIO_PIN_9
#define END_STOP_SYRINGE_1_GPIO_Port GPIOE
#define END_STOP_SYRINGE_1_EXTI_IRQn EXTI9_5_IRQn
#define SLEEP_NEEDLE_Pin GPIO_PIN_10
#define SLEEP_NEEDLE_GPIO_Port GPIOE
#define END_STOP_SYRINGE_2_Pin GPIO_PIN_11
#define END_STOP_SYRINGE_2_GPIO_Port GPIOE
#define END_STOP_SYRINGE_2_EXTI_IRQn EXTI15_10_IRQn
#define RESET_NEEDLE_Pin GPIO_PIN_12
#define RESET_NEEDLE_GPIO_Port GPIOE
#define END_STOP_NEEDLE_2_Pin GPIO_PIN_13
#define END_STOP_NEEDLE_2_GPIO_Port GPIOE
#define END_STOP_NEEDLE_2_EXTI_IRQn EXTI15_10_IRQn
#define MS3_NEEDLE_Pin GPIO_PIN_14
#define MS3_NEEDLE_GPIO_Port GPIOE
#define MS2_NEEDLE_Pin GPIO_PIN_15
#define MS2_NEEDLE_GPIO_Port GPIOE
#define MS1_NEEDLE_Pin GPIO_PIN_10
#define MS1_NEEDLE_GPIO_Port GPIOB
#define ENABLE_NEEDLE_Pin GPIO_PIN_11
#define ENABLE_NEEDLE_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define STEP_NEEDLE_Pin GPIO_PIN_12
#define STEP_NEEDLE_GPIO_Port GPIOD
#define RESET_SYRINGE_Pin GPIO_PIN_14
#define RESET_SYRINGE_GPIO_Port GPIOD
#define SLEEP_SYRINGE_Pin GPIO_PIN_15
#define SLEEP_SYRINGE_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define STEP_SYRINGE_Pin GPIO_PIN_15
#define STEP_SYRINGE_GPIO_Port GPIOA
#define FAN_OUT_Pin GPIO_PIN_6
#define FAN_OUT_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define DIR_SYRINGE_Pin GPIO_PIN_8
#define DIR_SYRINGE_GPIO_Port GPIOB
#define ENABLE_SYRINGE_Pin GPIO_PIN_9
#define ENABLE_SYRINGE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
