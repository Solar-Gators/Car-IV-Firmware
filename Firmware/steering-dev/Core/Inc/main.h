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
#define D7_Pin GPIO_PIN_13
#define D7_GPIO_Port GPIOC
#define D5_Pin GPIO_PIN_15
#define D5_GPIO_Port GPIOC
#define D4_Pin GPIO_PIN_0
#define D4_GPIO_Port GPIOH
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOH
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_2
#define D0_GPIO_Port GPIOC
#define Backlight_PWM_Pin GPIO_PIN_3
#define Backlight_PWM_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_0
#define RESET_GPIO_Port GPIOA
#define RD_Pin GPIO_PIN_1
#define RD_GPIO_Port GPIOA
#define WR_Pin GPIO_PIN_2
#define WR_GPIO_Port GPIOA
#define C_D_Pin GPIO_PIN_3
#define C_D_GPIO_Port GPIOA
#define Display_CS_Pin GPIO_PIN_4
#define Display_CS_GPIO_Port GPIOC
#define STM_OK_Pin GPIO_PIN_0
#define STM_OK_GPIO_Port GPIOB
#define ERROR_Pin GPIO_PIN_1
#define ERROR_GPIO_Port GPIOB
#define CAN_FAULT_Pin GPIO_PIN_2
#define CAN_FAULT_GPIO_Port GPIOB
#define MC_Btn_Pin GPIO_PIN_12
#define MC_Btn_GPIO_Port GPIOB
#define MC_Btn_EXTI_IRQn EXTI15_10_IRQn
#define Horn_Btn_Pin GPIO_PIN_13
#define Horn_Btn_GPIO_Port GPIOB
#define Horn_Btn_EXTI_IRQn EXTI15_10_IRQn
#define RGN_Btn_Pin GPIO_PIN_14
#define RGN_Btn_GPIO_Port GPIOB
#define RGN_Btn_EXTI_IRQn EXTI15_10_IRQn
#define Mode_Btn_Pin GPIO_PIN_15
#define Mode_Btn_GPIO_Port GPIOB
#define Mode_Btn_EXTI_IRQn EXTI15_10_IRQn
#define LT_Btn_Pin GPIO_PIN_6
#define LT_Btn_GPIO_Port GPIOC
#define LT_Btn_EXTI_IRQn EXTI9_5_IRQn
#define PTT_Btn_Pin GPIO_PIN_7
#define PTT_Btn_GPIO_Port GPIOC
#define PTT_Btn_EXTI_IRQn EXTI9_5_IRQn
#define Cminus_Btn_Pin GPIO_PIN_8
#define Cminus_Btn_GPIO_Port GPIOC
#define Cplus_Btn_Pin GPIO_PIN_9
#define Cplus_Btn_GPIO_Port GPIOC
#define Cplus_Btn_EXTI_IRQn EXTI9_5_IRQn
#define RT_Btn_Pin GPIO_PIN_8
#define RT_Btn_GPIO_Port GPIOA
#define RT_Btn_EXTI_IRQn EXTI9_5_IRQn
#define PV_Btn_Pin GPIO_PIN_9
#define PV_Btn_GPIO_Port GPIOA
#define D_C_Pin GPIO_PIN_10
#define D_C_GPIO_Port GPIOA
#define TRIP_Pin GPIO_PIN_15
#define TRIP_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_5
#define SD_CS_GPIO_Port GPIOB
#define SD_Detect_Pin GPIO_PIN_6
#define SD_Detect_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
