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

/* Initialize ADCs */
ADS7138 adcs[3] = {ADS7138(&hi2c4, 0x10), 
                    ADS7138(&hi2c4, 0x13), 
                    ADS7138(&hi2c4, 0x14)};

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
    for (int i = 0; i < 3; i++) {
        // Initialize ADC
        if (adcs[i].Init() != HAL_OK)
            Logger::LogError("ADC %d init failed", i);
        else
            Logger::LogInfo("ADC %d init success", i);

        // Set all ADC pins to analog inputs
        // TODO: This may not be necessary, should be set correctly on reset
        if (adcs[i].ConfigurePinMode(0x0) != HAL_OK)
            Logger::LogError("ADC %d pin configuration failed", i);

        // Set ADCs to manual conversion mode
        if (adcs[i].ConfigureOpmode(false, ConvMode_Type::MANUAL) != HAL_OK)
            Logger::LogError("ADC %d configure opmode failed", i);
    }
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