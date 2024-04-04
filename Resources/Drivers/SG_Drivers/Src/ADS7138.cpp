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