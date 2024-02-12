#include "user.hpp"

#include "threads.h"


extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" SPI_HandleTypeDef hspi1;

/* Initialize CAN frames and devices */

CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

/* Initialize DACs */
DACx311 throttle_dac = DACx311(&hspi1, THROTTLE_CS_GPIO_Port, THROTTLE_CS_Pin);
DACx311 regen_dac = DACx311(&hspi1, REGEN_CS_GPIO_Port, REGEN_CS_Pin);


void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddRxMessage(&io_test_frame, IoMsgCallback);
    CANController::AddRxMessage(&motor_control_frame, MotorUpdateCallback);
    CANController::AddFilterAll();
    CANController::Start();

    // Test DAC
    while (1) {
        throttle_dac.SetValue(0x0FFF);
        regen_dac.SetValue(0x0FFF);
        HAL_Delay(1000);
        throttle_dac.SetValue(0x0000);
        regen_dac.SetValue(0x0000);
        HAL_Delay(1000);
    }
}

void SetMotorState(bool state) {
    HAL_GPIO_WritePin(MC_MAIN_CTRL_GPIO_Port, MC_MAIN_CTRL_Pin, (GPIO_PinState)state);
}

void SetMotorMode(bool mode) {
    HAL_GPIO_WritePin(MC_PE_CTRL_GPIO_Port, MC_PE_CTRL_Pin, (GPIO_PinState)mode);
}

void SetMotorDirection(bool direction) {
    HAL_GPIO_WritePin(MC_FR_CTRL_GPIO_Port, MC_FR_CTRL_Pin, (GPIO_PinState)direction);
}

void SetThrottle(uint16_t value) {
    throttle_dac.SetValue(value);
}

void SetRegen(uint16_t value) {
    regen_dac.SetValue(value);
}