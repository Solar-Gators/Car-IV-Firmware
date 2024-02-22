#include "ADS7138.hpp"

ADS7138::ADS7138(I2C_HandleTypeDef *phi2c, uint8_t address) {
    this->_phi2c = phi2c;
    this->_address = address;
}

HAL_StatusTypeDef ADS7138::ReadReg(uint8_t reg, uint8_t *data) {
    uint8_t txData[2] = {ADS7138_OPCODE_READ, reg};

    if (HAL_I2C_Master_Transmit(_phi2c, _address, txData, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(_phi2c, _address, data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ADS7138::WriteReg(uint8_t reg, uint8_t data) {
    uint8_t txData[3] = {ADS7138_OPCODE_WRITE, reg, data};

    if (HAL_I2C_Master_Transmit(_phi2c, _address, txData, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}