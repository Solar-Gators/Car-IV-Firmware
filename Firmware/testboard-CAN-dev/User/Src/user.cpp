#include "user.hpp"
#include "threads.hpp"

/*
 * This program sends a simple CAN message between two testboards.
 * Load the program onto two testboards and connect them together. 
 * Pressing the user button will toggle the OK LED on the other testboard.
 * Long pressing the user button will toggle the error LED on the other testboard.
 */

extern "C" void CPP_UserSetup(void);
extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;


// Initialize User button
Button user_button = Button(BTN1_GPIO_Port, BTN1_Pin, 50, GPIO_PIN_SET, false);

// Initialize CAN frames and devices
CANFrame io_msg = CANFrame(0x37, CAN_ID_STD, CAN_RTR_DATA, 8, IoMsgCallback);
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);


void CPP_UserSetup(void) {
    // Test that timer priorities are configured correctly
    HAL_Delay(1);

    // Setup button
    user_button.RegisterNormalPressCallback(ButtonSingleCallback);
    user_button.RegisterLongPressCallback(ButtonLongCallback);
    //user_button.RegisterDoublePressCallback(DoubleCallback);

    // Initialize CAN controller
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddRxMessage(&io_msg);
    CANController::AddFilterAll();
    CANController::Start();

    // Start heartbeat thread
    osTimerStart(periodic_thread_id, 1000);
}