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
#include "stm32l4xx_hal.h"

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
#define HORN_Pin GPIO_PIN_13
#define HORN_GPIO_Port GPIOC
#define FAN_Pin GPIO_PIN_0
#define FAN_GPIO_Port GPIOC
#define HEADLIGHT_Pin GPIO_PIN_1
#define HEADLIGHT_GPIO_Port GPIOC
#define FR_LIGHT_Pin GPIO_PIN_2
#define FR_LIGHT_GPIO_Port GPIOC
#define FL_LIGHT_Pin GPIO_PIN_3
#define FL_LIGHT_GPIO_Port GPIOC
#define MC_FR_Pin GPIO_PIN_0
#define MC_FR_GPIO_Port GPIOA
#define MC_PE_Pin GPIO_PIN_1
#define MC_PE_GPIO_Port GPIOA
#define MC_MAIN_Pin GPIO_PIN_2
#define MC_MAIN_GPIO_Port GPIOA
#define REGEN_CS_Pin GPIO_PIN_3
#define REGEN_CS_GPIO_Port GPIOA
#define THROTTLE_CS_Pin GPIO_PIN_4
#define THROTTLE_CS_GPIO_Port GPIOA
#define ERROR_LED_Pin GPIO_PIN_4
#define ERROR_LED_GPIO_Port GPIOC
#define OK_LED_Pin GPIO_PIN_5
#define OK_LED_GPIO_Port GPIOC
#define NWC_Pin GPIO_PIN_0
#define NWC_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_1
#define IMU_INT_GPIO_Port GPIOB
#define RL_LIGHT_Pin GPIO_PIN_14
#define RL_LIGHT_GPIO_Port GPIOB
#define RR_LIGHT_Pin GPIO_PIN_15
#define RR_LIGHT_GPIO_Port GPIOB
#define RC_LIGHT_Pin GPIO_PIN_6
#define RC_LIGHT_GPIO_Port GPIOC
#define STRB_LIGHT_Pin GPIO_PIN_7
#define STRB_LIGHT_GPIO_Port GPIOC
#define KILL_SW_Pin GPIO_PIN_8
#define KILL_SW_GPIO_Port GPIOC
#define MPPT_PRECHARGE_Pin GPIO_PIN_9
#define MPPT_PRECHARGE_GPIO_Port GPIOC
#define MPPT_CONTACTOR_Pin GPIO_PIN_8
#define MPPT_CONTACTOR_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
