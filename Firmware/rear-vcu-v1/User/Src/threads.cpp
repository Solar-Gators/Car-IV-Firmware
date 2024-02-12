#include "threads.h"

void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, io_test_frame.GetOkLed());
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, io_test_frame.GetErrorLed());
}

void MotorUpdateCallback(uint8_t *data) {
    SetMotorState(motor_control_frame.StatusFlags() & MOTOR_MAIN_EN_FLAG);
    SetMotorMode(motor_control_frame.StatusFlags() & MOTOR_ECO_FLAG);
    SetMotorDirection(motor_control_frame.StatusFlags() & MOTOR_REVERSE_FLAG);
    SetThrottle(motor_control_frame.GetThrottleVal());
    SetRegen(motor_control_frame.GetRegenVal());    
}