#include "ADS7138.hpp"

ADS7138::ADS7138(I2C_HandleTypeDef *phi2c, uint8_t address) {
    this->_phi2c = phi2c;
    this->_address = address;
}

HAL_StatusTypeDef ADS7138::Init() {
    // Soft reset ADC
    uint8_t generalCfg = ADS7138_GENERAL_CFG_RST;

    if (WriteReg(ADS7138_Register::GENERAL_CFG, generalCfg) != HAL_OK)
        return HAL_ERROR;

    // Wait for reset to complete
    while (ReadReg(ADS7138_Register::GENERAL_CFG, &generalCfg) != HAL_OK);

    uint8_t system_status;
    ReadReg(ADS7138_Register::SYSTEM_STATUS, &system_status);

    // Check that bit 7 is set to 1
    if ((system_status & 0x80) == 0) {
        return HAL_ERROR;
    }

    // Check that I2C is not in high-speed mode
    if ((system_status & 0x10) != 0) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::TestI2C() {
    // Turn on fixed pattern mode
    uint8_t dataCfg = 0;
    if (ReadReg(ADS7138_Register::DATA_CFG, &dataCfg) != HAL_OK)
        return HAL_ERROR;
    dataCfg |= 0x80;

    if (WriteReg(ADS7138_Register::DATA_CFG, dataCfg) != HAL_OK)
        return HAL_ERROR;

    // Read channel 0 LSB
    uint8_t data = 0;
    if (ReadReg(ADS7138_Register::RECENT_CH0_LSB, &data) != HAL_OK)
        return HAL_ERROR;

    Logger::LogInfo("ADS7138 TestI2C: Channel 0 LSB: %d", data);

    // Read channel 0 MSB
    data = 0;
    if (ReadReg(ADS7138_Register::RECENT_CH0_MSB, &data) != HAL_OK)
        return HAL_ERROR;

    Logger::LogInfo("ADS7138 TestI2C: Channel 0 MSB: %d", data);

    // Turn off fixed pattern mode
    dataCfg &= 0x7F;
    if (WriteReg(ADS7138_Register::DATA_CFG, dataCfg) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::SequenceAll() {
    // Enable all channels in scanning sequence
    uint8_t seq_ch_sel = 0xFF;
    if (WriteReg(ADS7138_Register::SEQUENCE_CFG, seq_ch_sel) != HAL_OK)
        return HAL_ERROR;

    // Enable auto-sequence mode
    uint8_t seq_cfg = 0x1;
    if (WriteReg(ADS7138_Register::SEQUENCE_CFG, seq_cfg) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Configure the data register
 * @param fix_pattern Enable fixed pattern mode
 * @param append_type Append channel id and/or status bits to data
*/
HAL_StatusTypeDef ADS7138::ConfigureData(bool fix_pattern, DataCfg_AppendType append_type) {
    uint8_t dataCfg = 0;

    if (fix_pattern)
        dataCfg |= ADS7138_DATA_CFG_FIX_PAT;

    dataCfg |= static_cast<uint8_t>(append_type) << 4;

    if (WriteReg(ADS7138_Register::DATA_CFG, dataCfg) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Configure the oversampling rate
 * @param osr_cfg Oversampling rate enum
*/
HAL_StatusTypeDef ADS7138::ConfigureOversampling(OsrCfg_Type osr_cfg) {
    uint8_t osrCfg = static_cast<uint8_t>(osr_cfg);

    if (WriteReg(ADS7138_Register::OSR_CFG, osrCfg) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::StartSequence() {
    // Set start sequence bit, set sequence mode to auto sequence mode
    uint8_t seq_cfg = ADS7138_SEQUENCE_CFG_SEQ_START | 0x1;

    if (WriteReg(ADS7138_Register::SEQUENCE_CFG, seq_cfg) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Configure ADC operational mode
 * @param conv_on_err If set to true, CRC error will change all inputs to analog and pause channel sequencing
 * @param conv_mode ADC conversion initiator mode, manual, autonomous, or turbo mode
*/
HAL_StatusTypeDef ADS7138::ConfigureOpmode(bool conv_on_err, ConvMode_Type conv_mode) {
    uint8_t generalCfg = 0;

    if (conv_on_err)
        generalCfg |= ADS7138_GENERAL_CFG_DWC_EN;

    generalCfg |= static_cast<uint8_t>(conv_mode);

    if (WriteReg(ADS7138_Register::GENERAL_CFG, generalCfg) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Configure ADC pin modes as analog inputs or GPIOs
 * @param pin_mode Pin mode configuration, bit set for GPIO, bit clear for analog input for each channel
*/
HAL_StatusTypeDef ADS7138::ConfigurePinMode(uint8_t pin_mode) {
    if (WriteReg(ADS7138_Register::PIN_CFG, pin_mode) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Select a channel for manual conversion (ADC must be in manual mode)
 * @param channel Channel to select (0-7)
*/
HAL_StatusTypeDef ADS7138::ManualSelectChannel(uint8_t channel) {
    if (channel > 7)
        return HAL_ERROR;

    if (WriteReg(ADS7138_Register::MANUAL_CH_SEL, channel) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::StartConversion() {
    SetRegBits(ADS7138_Register::GENERAL_CFG, ADS7138_GENERAL_CFG_CNVST);

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::ReadChannel(uint8_t channel, uint16_t *data) {
    // Read channel data
    uint8_t reg = static_cast<uint8_t>(ADS7138_Register::RECENT_CH0_LSB) + (channel * 2);
    uint8_t dataLSB = 0;
    uint8_t dataMSB = 0;

    // Read LSB register
    if (ReadReg(static_cast<ADS7138_Register>(reg), &dataLSB) != HAL_OK)
        return HAL_ERROR;

    // Read MSB register
    if (ReadReg(static_cast<ADS7138_Register>(reg + 1), &dataMSB) != HAL_OK)
        return HAL_ERROR;

    *data = (dataMSB << 8) | dataLSB;

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::ReadReg(ADS7138_Register reg, uint8_t *data) {
    uint8_t txData[2] = {static_cast<uint8_t>(ADS7138_Opcode::READ), 
                        static_cast<uint8_t>(reg)};

    if (HAL_I2C_Master_Transmit(_phi2c, _address << 1, txData, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(_phi2c, _address << 1, data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::WriteReg(ADS7138_Register reg, uint8_t data) {
    uint8_t txData[3] = {static_cast<uint8_t>(ADS7138_Opcode::WRITE), 
                        static_cast<uint8_t>(reg), 
                        data};

    if (HAL_I2C_Master_Transmit(_phi2c, _address << 1, txData, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
        Logger::LogError("ADS7138 WriteReg: I2C transmit failed");
    }

    return HAL_OK;
}

/**
  * @brief Set bits in a register
  * @param reg Register to modify
  * @param mask Bits to set
 */
HAL_StatusTypeDef ADS7138::SetRegBits(ADS7138_Register reg, uint8_t mask) {
    uint8_t data;
    if (ReadReg(reg, &data) != HAL_OK)
        return HAL_ERROR;

    data |= mask;

    if (WriteReg(reg, data) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief Set bits in a register
  * @param reg Register to modify
  * @param mask Bits to clear
 */
HAL_StatusTypeDef ADS7138::ClearRegBits(ADS7138_Register reg, uint8_t mask) {
    uint8_t data;
    if (ReadReg(reg, &data) != HAL_OK)
        return HAL_ERROR;

    data &= ~mask;

    if (WriteReg(reg, data) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}