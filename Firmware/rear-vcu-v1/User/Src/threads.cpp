#include "threads.h"

void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, IoTestFrame::GetOkLed());
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, IoTestFrame::GetErrorLed());
}

void MotorUpdateCallback(uint8_t *data) {
    SetThrottle(MotorControlFrame::GetThrottleVal());
    SetRegen(MotorControlFrame::GetRegenVal());    
}