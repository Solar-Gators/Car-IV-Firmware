#include "user.hpp"
#include "threads.hpp"

extern "C" void CPP_UserSetup(void);


Button user_button = Button(BTN1_GPIO_Port, BTN1_Pin, 50, GPIO_PIN_SET, false);

CANFrame io_msg = CANFrame(0x37, CAN_ID_STD, CAN_RTR_DATA, 8);
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);


void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(2);

    user_button.RegisterNormalPressCallback(ButtonCallback);
    user_button.RegisterLongPressCallback(LongCallback);
    //user_button.RegisterDoublePressCallback(DoubleCallback);

    // Initialize CAN things
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddFilterAll();
    CANController::Start();

    osTimerStart(periodic_thread_id, 1000);
}

void ButtonCallback(void) {
    HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
}

void LongCallback(void) {
    HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
}

void DoubleCallback(void) {
    HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
}