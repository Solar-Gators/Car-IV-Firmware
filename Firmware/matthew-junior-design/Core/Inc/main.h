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
#define ERROR_LED_Pin GPIO_PIN_0
#define ERROR_LED_GPIO_Port GPIOA
#define OK_LED_Pin GPIO_PIN_1
#define OK_LED_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_3
#define JOY_Y_GPIO_Port GPIOA
#define TFTBL_Pin GPIO_PIN_4
#define TFTBL_GPIO_Port GPIOA
#define PM_ALERT_Pin GPIO_PIN_2
#define PM_ALERT_GPIO_Port GPIOB
#define SDCS_Pin GPIO_PIN_10
#define SDCS_GPIO_Port GPIOB
#define TFTCS_Pin GPIO_PIN_11
#define TFTCS_GPIO_Port GPIOB
#define JOY_BTN_Pin GPIO_PIN_12
#define JOY_BTN_GPIO_Port GPIOB
#define JOY_BTN_EXTI_IRQn EXTI15_10_IRQn
#define TFTDC_Pin GPIO_PIN_13
#define TFTDC_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_10
#define BTN1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE   hspi1
#define SD_CS_GPIO_Port SDCS_GPIO_Port
#define SD_CS_Pin       SDCS_Pin

#define AUDIO_TIM_HANDLE htim5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
