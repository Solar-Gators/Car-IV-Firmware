#include "user.hpp"

#include "threads.h"


// BMS to-do
// - Voltage measurement (done)
// - Temperature measurement (in progress)
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

/* Initialize CAN frames and devices */
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

/* Initialize BMS */
BQ76952 bms;

/* Initialize ADC */
ADS7138 adc1 = ADS7138(&hi2c4, 0x13);
ADS7138 adc2 = ADS7138(&hi2c4, 0x14);

void DefaultOutputs() {
    // Turn off thermistor amplifiers
    SetAmplifierState(false);
}

void CAN_Modules_Init() {
    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddFilterAll();
    CANController::Start();
}

void ADC_Modules_Init() {
    // Initialize ADCs
    // if (adc1.Init() != HAL_OK)
    //     Logger::LogError("ADC init failed");
    // else
    //     Logger::LogInfo("ADC init success");

    if (adc2.Init() != HAL_OK)
        Logger::LogError("ADC init failed");
    else
        Logger::LogInfo("ADC init success");

    adc2.TestI2C();
}

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    DefaultOutputs();
    
    CAN_Modules_Init();

    ADC_Modules_Init();

    // Initialize BMS
    if (bms.Init(&hi2c3) != HAL_OK)
        Logger::LogError("BMS init failed");
    else
        Logger::LogInfo("BMS init success");

    // Configure REG1 voltage to 3.3v

    // Start fan PWM
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    ThreadsStart();
}

/* Controls power supply to thermistor amplifier ICs */
void SetAmplifierState(bool state) {
    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}