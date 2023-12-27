#include "threads.hpp"

/* Externs */
extern Button user_button;
extern CANFrame io_msg;
extern CANDevice candev1;
extern CANDevice candev2;

/* Periodic thread definitions */
osTimerAttr_t periodic_thread_attr = {
    .name = "Periodic Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_thread_id = osTimerNew((osTimerFunc_t)PeriodicThread, osTimerPeriodic, NULL, &periodic_thread_attr);


/* Thread function definitions */
void PeriodicThread(void *argument) {
    Logger::LogInfo("Still alive!\n");
}

/* Callbacks */
void ButtonSingleCallback(void) {
    Logger::LogInfo("Button Single Press\n");
    io_msg.data[0] = user_button.GetToggleState();
    CANController::Send(&io_msg);
}

void ButtonLongCallback(void) {
    Logger::LogInfo("Button Long Press\n");
    io_msg.data[1] = user_button.GetLongToggleState();
    CANController::Send(&io_msg);
}

void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, (GPIO_PinState)data[0]);
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, (GPIO_PinState)data[1]);
}