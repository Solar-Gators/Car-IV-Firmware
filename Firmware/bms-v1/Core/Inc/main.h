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
#define BMS_ALERT_Pin GPIO_PIN_3
#define BMS_ALERT_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_0
#define ERROR_LED_GPIO_Port GPIOA
#define OK_LED_Pin GPIO_PIN_1
#define OK_LED_GPIO_Port GPIOA
#define MIO1_Pin GPIO_PIN_5
#define MIO1_GPIO_Port GPIOA
#define MIO2_Pin GPIO_PIN_6
#define MIO2_GPIO_Port GPIOA
#define AMP_EN_Pin GPIO_PIN_7
#define AMP_EN_GPIO_Port GPIOA
#define CURRENT_EN_Pin GPIO_PIN_4
#define CURRENT_EN_GPIO_Port GPIOC
#define CONTACTOR2_CTRL_Pin GPIO_PIN_5
#define CONTACTOR2_CTRL_GPIO_Port GPIOC
#define CONTACTOR1_CTRL_Pin GPIO_PIN_0
#define CONTACTOR1_CTRL_GPIO_Port GPIOB
#define CONTACTOR_SOURCE_SEL_Pin GPIO_PIN_1
#define CONTACTOR_SOURCE_SEL_GPIO_Port GPIOB
#define CONTACTOR_SOURCE_ST_Pin GPIO_PIN_2
#define CONTACTOR_SOURCE_ST_GPIO_Port GPIOB
#define FAN_SEL_1_Pin GPIO_PIN_12
#define FAN_SEL_1_GPIO_Port GPIOB
#define FAN_SEL_0_Pin GPIO_PIN_15
#define FAN_SEL_0_GPIO_Port GPIOB
#define FAN1_PWM_Pin GPIO_PIN_6
#define FAN1_PWM_GPIO_Port GPIOC
#define FAN2_PWM_Pin GPIO_PIN_7
#define FAN2_PWM_GPIO_Port GPIOC
#define FAN3_PWM_Pin GPIO_PIN_8
#define FAN3_PWM_GPIO_Port GPIOC
#define FAN4_PWM_Pin GPIO_PIN_9
#define FAN4_PWM_GPIO_Port GPIOC
#define FAN_TACH_Pin GPIO_PIN_8
#define FAN_TACH_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
