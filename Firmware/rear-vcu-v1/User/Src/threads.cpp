#include "threads.h"

osTimerAttr_t periodic_timer_attr = {
    .name = "Periodic Task 1",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_timer_id = osTimerNew((osThreadFunc_t)SendMitsubaRequest, osTimerPeriodic, NULL, &periodic_timer_attr);

void Start() {
    osTimerStart(periodic_timer_id, 500);
}

void SendMitsubaRequest() {
    // Send a request to the Mitsuba motor controller
    CANController::Send(&MitsubaRequestFrame::Instance());

    // Heartbeat LED
    HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
}

void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, IoTestFrame::GetOkLed());
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, IoTestFrame::GetErrorLed());
}

void MotorUpdateCallback(uint8_t *data) {
    //SetMotorState(DriverControlsFrame0::GetMotorEnable());
    SetMotorMode(DriverControlsFrame0::GetDriveMode());
    SetMotorDirection(DriverControlsFrame0::GetDriveDirection());
    SetThrottle(DriverControlsFrame0::GetThrottleVal());
    SetRegen(DriverControlsFrame0::GetRegenVal());   

    Logger::LogInfo("Motor current: %d", MitsubaFrame0::Instance().GetMotorCurrent()); 
}

void MitsubaCallback(uint8_t *data) {
    // Update the Mitsuba motor controller state
    Logger::LogInfo("Mitsuba callback called\n");
}