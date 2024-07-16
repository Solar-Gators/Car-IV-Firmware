#include "user.hpp"

#include "threads.h"


// BMS to-do
// - Voltage measurement (done)
// - Temperature measurement (done)
// - CAN frame (in progress)
// - SOC calculation (in progress)
// - Current measurement
// - Contactor control
// - Fan control
// - Balancing control

extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" I2C_HandleTypeDef hi2c3;
extern "C" I2C_HandleTypeDef hi2c4;
extern "C" TIM_HandleTypeDef htim3;

/* Global data */

/* Initialize CAN frames and devices */
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

void DefaultOutputs() {

    // TODO: Debug only
    // HAL_GPIO_WritePin(CONTACTOR1_CTRL_GPIO_Port, CONTACTOR1_CTRL_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(CONTACTOR2_CTRL_GPIO_Port, CONTACTOR2_CTRL_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(CONTACTOR3_CTRL_GPIO_Port, CONTACTOR3_CTRL_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(CONTACTOR4_CTRL_GPIO_Port, CONTACTOR4_CTRL_Pin, GPIO_PIN_SET);
}

void CAN_Modules_Init() {
    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddRxMessage(&ProhelionCurrentSenseFrame::Instance(), ProhelionCurrentCallback);
    CANController::AddFilterAll();
    CANController::Start();
}

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    DefaultOutputs();
    
    CAN_Modules_Init();

    ThreadsStart();
}

/* Sets contactor */
void SetContactorState(uint8_t contactor, bool state) {
    switch (contactor) {
        case 1:
            HAL_GPIO_WritePin(CONTACTOR1_CTRL_GPIO_Port, 
                              CONTACTOR1_CTRL_Pin, 
                              state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(CONTACTOR2_CTRL_GPIO_Port, 
                              CONTACTOR2_CTRL_Pin, 
                              state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(CONTACTOR3_CTRL_GPIO_Port, 
                              CONTACTOR3_CTRL_Pin, 
                              state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(CONTACTOR4_CTRL_GPIO_Port, 
                              CONTACTOR4_CTRL_Pin, 
                              state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        default:
            Logger::LogError("Invalid contactor number");
    }
}