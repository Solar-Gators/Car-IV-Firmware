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
    CANController::AddFilterAll();
    CANController::Start();
}

void ADC_Modules_Init() {
    // Common for all ADCs
    for (int i = 0; i < 3; i++) {
        // Initialize ADC and test I2C
        if (adcs[i].Init() != HAL_OK || adcs[i].TestI2C() != HAL_OK)
            Logger::LogError("ADC %d init failed", i);
        else
            Logger::LogInfo("ADC %d init success", i);

        // Configure all channels as ADC inputs
        if (adcs[i].ConfigurePinMode(0x0) != HAL_OK)
            Logger::LogError("ADC %d configure pin mode failed", i);

        // Configure auto-sequence
        if (adcs[i].ConfigureSequenceMode(SeqMode_Type::AUTO) != HAL_OK)
            Logger::LogError("ADC %d configure sequence mode failed", i);

        // Configure sampling rate to 166.7 kSPS
        if (adcs[i].ConfigureOpmode(false, 
                                    ConvMode_Type::AUTONOMOUS, 
                                    Osc_Type::HIGH_SPEED, 
                                    0b0101) != HAL_OK)
            Logger::LogError("ADC %d configure opmode failed", i);

        // Configure oversampling to 16x
        if (adcs[i].ConfigureOversampling(OsrCfg_Type::OSR_32) != HAL_OK)
            Logger::LogError("ADC %d configure oversampling failed", i);

        // Enable statistics
        if (adcs[i].ConfigureStatistics(true) != HAL_OK)
            Logger::LogError("ADC %d configure statistics failed", i);

        // For all ADCs, append channel ID to data
        if (adcs[i].ConfigureData(false, DataCfg_AppendType::ID) != HAL_OK)
            Logger::LogError("ADC %d configure data failed", i);
    }

    // For adc0, sequence channels 5 and 7 for current measurement
    if (adcs[0].ConfigureSequence(0b10100000) != HAL_OK)
        Logger::LogError("ADC 0 configure sequence failed");

    // For adc1 and adc2, sequence all channels
    if (adcs[1].ConfigureSequenceAll() != HAL_OK)
        Logger::LogError("ADC 1 configure sequence failed");
    if (adcs[2].ConfigureSequenceAll() != HAL_OK)
        Logger::LogError("ADC 2 configure sequence failed");
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

    // Set BMS to always auto balance
    if (bms.ChangeBalancingStatus(false, true, true, true) != HAL_OK)
        Logger::LogError("BMS set balance all failed");

    // Configure REG1 voltage to 3.3v

    // Start fan PWM
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    

    ThreadsStart();
}

/* Controls power supply to thermistor amplifier ICs */
void SetAmplifierState(bool state) {
    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
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