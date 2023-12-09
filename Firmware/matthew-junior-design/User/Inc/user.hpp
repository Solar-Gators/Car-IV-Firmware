#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "ST7789.hpp"
#include "STM32SAM.h"
#include "fatfs.h"

extern "C" SPI_HandleTypeDef hspi1;     // display
extern "C" TIM_HandleTypeDef htim4;     // debounce timer
extern "C" TIM_HandleTypeDef htim5;     // sound timer
extern "C" ADC_HandleTypeDef hadc1;     // joystick y-axis
extern "C" ADC_HandleTypeDef hadc2;     // joystick x-axis
extern "C" DAC_HandleTypeDef hdac1;     // backlight

void DisplayBanner(const char* text);
uint16_t GetJoyXY();
void SetBacklight(uint8_t brightness);  // 0-50
void SetVolume(uint8_t volume);         // 0-50

extern char text_buffer[20][100];               // Text buffer

extern ST7789 display;

extern STM32SAM voice;


#endif /* USER_HPP_ */
