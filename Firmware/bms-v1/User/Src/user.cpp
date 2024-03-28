#include "user.hpp"

#include "threads.h"


extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" I2C_HandleTypeDef hi2c3;
extern "C" I2C_HandleTypeDef hi2c4;
extern "C" TIM_HandleTypeDef htim3;

/* Initialize CAN frames and devices */
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

/* Initialize BMS */
BQ76952 bms;

/* Initialize ADC */
ADS7138 adc1 = ADS7138(&hi2c4, 0x13);
ADS7138 adc2 = ADS7138(&hi2c4, 0x14);

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddFilterAll();
    // CANController::Start();

    // Initialize BMS
    if (bms.Init(&hi2c3) != HAL_OK)
        Logger::LogError("BMS init failed");
    else
        Logger::LogInfo("BMS init success");

    // Initialize ADCs
    // if (adc1.Init() != HAL_OK)
    //     Logger::LogError("ADC init failed");
    // else
    //     Logger::LogInfo("ADC init success");

    // if (adc2.Init() != HAL_OK)
    //     Logger::LogError("ADC init failed");
    // else
    //     Logger::LogInfo("ADC init success");

    // Turn on amplifiers
    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_SET);

    // while (1) {
    //     adc1.TestI2C();
    //     adc2.TestI2C();
    // }

    // Configure REG1 voltage to 3.3v

    // Start fan PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Test LED
    while (1) {
        if (bms.ReadVoltages() != HAL_OK)
            Logger::LogError("BMS read failed");
        else
            Logger::LogInfo("BMS read success");

        Logger::LogInfo("Cell 1: %d mV", bms.GetCellVoltage(0));
        Logger::LogInfo("Cell 2: %d mV", bms.GetCellVoltage(1));
        Logger::LogInfo("Cell 3: %d mV", bms.GetCellVoltage(2));
        Logger::LogInfo("Pack: %d mV", bms.GetPackVoltage() * 10);
        
        HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
        HAL_Delay(1000);
    }

}