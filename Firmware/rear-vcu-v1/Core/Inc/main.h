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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MC_FR_CTRL_Pin GPIO_PIN_0
#define MC_FR_CTRL_GPIO_Port GPIOC
#define MC_PE_CTRL_Pin GPIO_PIN_1
#define MC_PE_CTRL_GPIO_Port GPIOC
#define MC_MAIN_CTRL_Pin GPIO_PIN_2
#define MC_MAIN_CTRL_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_0
#define ERROR_LED_GPIO_Port GPIOA
#define OK_LED_Pin GPIO_PIN_1
#define OK_LED_GPIO_Port GPIOA
#define REGEN_CS_Pin GPIO_PIN_3
#define REGEN_CS_GPIO_Port GPIOA
#define THROTTLE_CS_Pin GPIO_PIN_4
#define THROTTLE_CS_GPIO_Port GPIOA
#define MPPT_CONTACTOR_EN_Pin GPIO_PIN_4
#define MPPT_CONTACTOR_EN_GPIO_Port GPIOC
#define MPPT_PRE_CONTACTOR_EN_Pin GPIO_PIN_5
#define MPPT_PRE_CONTACTOR_EN_GPIO_Port GPIOC
#define RL_LIGHT_EN_Pin GPIO_PIN_6
#define RL_LIGHT_EN_GPIO_Port GPIOC
#define RR_LIGHT_EN_Pin GPIO_PIN_7
#define RR_LIGHT_EN_GPIO_Port GPIOC
#define RC_LIGHT_EN_Pin GPIO_PIN_8
#define RC_LIGHT_EN_GPIO_Port GPIOC
#define STRB_LIGHT_EN_Pin GPIO_PIN_9
#define STRB_LIGHT_EN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
