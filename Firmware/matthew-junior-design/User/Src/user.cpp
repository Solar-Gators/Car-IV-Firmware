#include "user.hpp"
#include "threads.h"

/* In this program, two tasks are generated. One is triggered by a periodic timer, which
 * expires every 500ms. The other is a regular thread, which is triggered by and event flag.
 * The periodic timer task sets the event flag, which causes the regular thread to exit
 * its wait state and execute. The regular thread then toggles the LED and prints a message.
 * 
 * CPP_UserSetup is called from main.cpp, and is where the user should put their setup code.
 * It is run before the RTOS scheduler is started.
*/

/* main.c functions */
extern "C" void CPP_UserSetup(void);
extern "C" void DebounceCallback(void);

ST7789 display = ST7789(&hspi1, TFTCS_GPIO_Port, TFTCS_Pin, 
                        TFTDC_GPIO_Port, TFTDC_Pin, 0, 0, 0, 0);

ReaderUI ui = ReaderUI();

STM32SAM voice = STM32SAM(5);

/* Global variables */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
uint32_t pressed_time = 0;              // Time button was pressed


/* Function definitions */
uint16_t GetJoyXY() {
    // Read joystick y-axis, simple ADC read
    uint32_t y_val;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    y_val = HAL_ADC_GetValue(&hadc1);

    // Read joystick x-axis, reconfigure GPIO
    GPIO_InitTypeDef GPIO_InitStruct0 = {};
    GPIO_InitStruct0.Pin = GPIO_PIN_2;
    GPIO_InitStruct0.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct0.Pull = GPIO_NOPULL;

    GPIO_InitTypeDef GPIO_InitStruct1 = {};
    GPIO_InitStruct1.Pin = GPIO_PIN_2;
    GPIO_InitStruct1.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct1.Pull = GPIO_NOPULL;
    GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct1.Alternate = GPIO_AF2_TIM5;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct0);

    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    uint32_t x_val = HAL_ADC_GetValue(&hadc2);

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);


    return ((x_val << 8) & 0xFF00) | (y_val & 0x00FF);
}

// Brightness input between 0 and 50
void SetBacklight(uint8_t brightness) {
    if (brightness > 50)
        brightness = 50;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, brightness+80);
}

void SetVolume(uint8_t volume) {
    if (volume > 50)
        volume = 50;

    // Reload goes from 255 to 2048, higher volume means lower reload
    uint32_t auto_reload = 14025 / (volume + 5);

    // Count goes crazy for some reason
    __HAL_TIM_SET_AUTORELOAD(&htim5, auto_reload);
    __HAL_TIM_SET_COUNTER(&htim5, auto_reload);
}

void MoveCursor(uint32_t source, uint32_t dest) {
    // Check boundary conditions
    if (source > 19)
        source = 19;
    if (dest > 19)
        dest = 19;

    // Blank out old cursor
    ui.DrawText(&FontStyle_Emulogic, ">", 30, 195 - (20 * source), ST7789_WHITE);

    // Draw new cursor
    ui.DrawText(&FontStyle_Emulogic, ">", 30, 195 - (20 * dest), ST7789_RED);
}

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Initialize display
    ui.Init();

    // Set backlight to max brightness
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    SetBacklight(50);

    // Say "power on"
    voice.setVoice(72, 72, 128, 128);
    voice.say("Power On!");
    voice.setPitch(64);

    // Attempt to mount SD card
    if(f_mount(&fs, "", 1) != FR_OK) {
        Logger::LogError("SD card mount failed\n");
        ui.DrawFastRectangle(0, 0, 320, 240, true);
        ui.DrawFastRectangle(50, 100, 220, 2, false);
        ui.DrawFastRectangle(50, 140, 220, 2, false);
        ui.DrawFastRectangle(50, 100, 2, 40, false);
        ui.DrawFastRectangle(268, 100, 2, 40, false);
        ui.DrawText(&FontStyle_Emulogic, "SD card not detected!", 75, 115, ST7789_RED);
        Error_Handler();
    }
    else
        Logger::LogInfo("SD card mount successful\n");

    // Start scheduler
    osTimerStart(joystick_timer_id, 200);
}

void DebounceCallback(void) {
    // If button is still pressed
    if (HAL_GPIO_ReadPin(JOY_BTN_GPIO_Port, JOY_BTN_Pin) == GPIO_PIN_RESET) {
        // If double press timer is running, register double press and reset timer
        if (pressed_time != 0 && HAL_GetTick() - pressed_time < 500) {
            osEventFlagsSet(regular_event, JOY_BTN_DOUBLE);
            pressed_time = 0;
        }
        // Register single press and start double press timer
        else {
            osEventFlagsSet(regular_event, JOY_BTN);
            pressed_time = HAL_GetTick();
        }
    }
}