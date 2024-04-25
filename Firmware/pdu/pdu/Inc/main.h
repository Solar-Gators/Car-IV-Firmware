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
#define AUX_ADC_Pin GPIO_PIN_0
#define AUX_ADC_GPIO_Port GPIOC
#define HORN_ADC_Pin GPIO_PIN_1
#define HORN_ADC_GPIO_Port GPIOC
#define SUPP_ADC_Pin GPIO_PIN_2
#define SUPP_ADC_GPIO_Port GPIOC
#define BUCK_ADC_Pin GPIO_PIN_3
#define BUCK_ADC_GPIO_Port GPIOC
#define AUX_FLT_Pin GPIO_PIN_4
#define AUX_FLT_GPIO_Port GPIOA
#define AUX_OFF_Pin GPIO_PIN_5
#define AUX_OFF_GPIO_Port GPIOA
#define HORN_FLT_Pin GPIO_PIN_6
#define HORN_FLT_GPIO_Port GPIOA
#define HORN_OFF_Pin GPIO_PIN_7
#define HORN_OFF_GPIO_Port GPIOA
#define SUPP_FLT_Pin GPIO_PIN_5
#define SUPP_FLT_GPIO_Port GPIOC
#define SUPP_OFF_Pin GPIO_PIN_0
#define SUPP_OFF_GPIO_Port GPIOB
#define BUCK_FLT_Pin GPIO_PIN_1
#define BUCK_FLT_GPIO_Port GPIOB
#define BUCK_OFF_Pin GPIO_PIN_2
#define BUCK_OFF_GPIO_Port GPIOB
#define ERROR_LED_Pin GPIO_PIN_10
#define ERROR_LED_GPIO_Port GPIOB
#define OK_LED_Pin GPIO_PIN_11
#define OK_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
