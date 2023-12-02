#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "ST7789.hpp"
#include "STM32SAM.h"

extern "C" SPI_HandleTypeDef hspi1;     // display
extern "C" TIM_HandleTypeDef htim4;     // debounce timer
extern "C" TIM_HandleTypeDef htim5;     // sound timer
extern "C" ADC_HandleTypeDef hadc1;     // joystick y-axis
extern "C" ADC_HandleTypeDef hadc2;     // joystick x-axis
extern "C" DAC_HandleTypeDef hdac1;     // backlight

uint16_t GetJoyXY();
void SetBacklight(uint8_t brightness);

extern ST7789 display;

extern STM32SAM voice;


#endif /* USER_HPP_ */
