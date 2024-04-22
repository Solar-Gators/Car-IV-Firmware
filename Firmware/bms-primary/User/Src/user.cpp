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
extern "C" TIM_HandleTypeDef htim1;
extern "C" TIM_HandleTypeDef htim2;
extern "C" TIM_HandleTypeDef htim3;
extern "C" TIM_HandleTypeDef htim16;

/* Global data */

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

    // Set contactors power source to supplemental battery
    SetContactorSource(ContactorSource_Type::MAIN);

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
    CANController::AddRxMessage(&VCUFrame0::Instance(), VCUFrameCallback);
    CANController::AddRxMessage(&BMSSecondaryFrame0::Instance());
    CANController::AddRxMessage(&BMSSecondaryFrame1::Instance());
    CANController::AddRxMessage(&BMSSecondaryFrame2::Instance());
    CANController::AddRxMessage(&BMSSecondaryFrame3::Instance());
    CANController::AddFilterAll();
    CANController::Start();
}

void ADC_Modules_Init() {
    for (int i = 0; i < 3; i++) {
        // Initialize ADC
        if (adcs[i].Init() != HAL_OK || adcs[i].TestI2C() != HAL_OK)
            Logger::LogError("ADC %d init failed", i);
        else
            Logger::LogInfo("ADC %d init success", i);

        // Set all ADCs to initiate conversion on request
        if (adcs[i].ConfigureOpmode(false, ConvMode_Type::MANUAL) != HAL_OK)
            Logger::LogError("ADC %d configure opmode failed", i);

        // For all ADCs, append channel ID to data
        if (adcs[i].ConfigureData(false, DataCfg_AppendType::ID) != HAL_OK)
            Logger::LogError("ADC %d configure data failed", i);
    }

    // For adc0, sequence channels 5, 7 for current sense
    if (adcs[0].AutoSelectChannels((0x1 << 5) | (0x1 << 7)) != HAL_OK)
       Logger::LogError("ADC 0 auto select channels failed");

    // For adc1, sequence all channels, all channels are thermistors
    if (adcs[1].AutoSelectChannels(0xFF) != HAL_OK)
        Logger::LogError("ADC 1 auto select channels failed");

    // For adc2, sequence all channels, all channels are thermistors
    if (adcs[2].AutoSelectChannels(0xFF) != HAL_OK)
        Logger::LogError("ADC 2 auto select channels failed");
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

/* Selects contactor power supply between main 12V and supplemental battery */
void SetContactorSource(ContactorSource_Type source) {
    HAL_GPIO_WritePin(CONTACTOR_SOURCE_SEL_GPIO_Port, 
                        CONTACTOR_SOURCE_SEL_Pin, 
                        static_cast<GPIO_PinState>(source));
}

/* Sets contactor */
void SetContactorState(uint8_t contactor, bool state) {
    switch (contactor) {
        case 1:
            HAL_GPIO_WritePin(CONTACTOR1_CTRL_GPIO_Port, CONTACTOR1_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(CONTACTOR2_CTRL_GPIO_Port, CONTACTOR2_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(CONTACTOR3_CTRL_GPIO_Port, CONTACTOR3_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(CONTACTOR4_CTRL_GPIO_Port, CONTACTOR4_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        default:
            Logger::LogError("Invalid contactor number");
    }
}

/* Converts raw ADC value to temperature in degrees C */
float ADCToTemp(uint16_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 1.0 / 1180;

    // Constant offset for linear estimator
    static constexpr float b = 19000.0 / 1180;

    // Convert ADC value to temperature
    return (float)adc_val * m + b;
}

/* Converts raw ADC value to current in A for low channel */
float ADCToCurrentL(uint16_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 0.001894;

    // Constant offset for linear estimator
    static constexpr float b = -62.87;

    // Convert ADC value to current
    return (float)adc_val * m + b;
}

/* Converts raw ADC value to current in A for high channel */
float ADCToCurrentH(uint16_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 0.007609;

    // Constant offset for linear estimator
    static constexpr float b = -252.4;

    // Convert ADC value to current
    return (float)adc_val * m + b;
}