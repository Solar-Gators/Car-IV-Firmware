#include "INA226.hpp"

HAL_StatusTypeDef INA226::Init(I2C_HandleTypeDef *hi2c){
    hi2c_ = hi2c;

    return HAL_I2C_IsDeviceReady(hi2c_, INA_I2C_ADDR, 10, 5);
}

HAL_StatusTypeDef INA226::GetShuntVoltage(){
    uint8_t buff[2];
    uint16_t val;
    HAL_StatusTypeDef status;

    status = ReadWordReg(INA_SHUNT_VOLTAGE, buff);
    if(status != HAL_OK) return status;

    val = (buff[1] << 8) | buff[0];

    shunt_voltage_ = val * 2.5 / 1e6;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::GetCurrent(){
    HAL_StatusTypeDef status = GetShuntVoltage();
    if(status != HAL_OK) return status;

    current_ = shunt_voltage_ / shunt_resistance_;
    return status;
}

HAL_StatusTypeDef INA226::GetBusVoltage(){
    uint8_t buff[2];
    uint16_t val;
    HAL_StatusTypeDef status;

    status = ReadWordReg(INA_BUS_VOLTAGE, buff);
    if(status != HAL_OK) return status;

    val = (buff[1] << 8) | buff[0];

    shunt_voltage_ = val * 2.5 / 1e6;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::SetShuntResistance(float ohms){
    shunt_resistance_ = ohms;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::Reset(){
    HAL_StatusTypeDef status;
    uint8_t reset_buff[3] = {INA_CONFIG, 0x80, 0x00}; // set first byte to be our register address, second two bytes is just top bit for reset

    status = HAL_I2C_Master_Transmit(hi2c_, INA_I2C_ADDR, reset_buff, 3, 1000); // reset the device
    if(status != HAL_OK) return status;

    status = SetConfig();
    
    return status;
}

HAL_StatusTypeDef INA226::SetConfig(){
    uint16_t config_settings;

    // Used to configure sample averages, bus and shunt voltage conversion time, operating mode
    // Use bitfields found on page 22 and 23 of https://www.ti.com/lit/ds/symlink/ina226.pdf?ts=1704213256231&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA226
    config_settings = 0b011 << 9 |
                        0b100 << 6 |
                        0b100 << 3 |
                        0b111;

    return WriteWordReg(INA_CONFIG, config_settings);
}

HAL_StatusTypeDef INA226::ReadWordReg(uint8_t reg_addr, uint8_t *data){
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c_, INA_I2C_ADDR, &reg_addr, 1, 1000);

    if(status != HAL_OK) return status;

    return HAL_I2C_Master_Receive(hi2c_, INA_I2C_ADDR, data, 2, 1000);
}

HAL_StatusTypeDef INA226::WriteWordReg(uint8_t reg_addr, uint16_t data){
    uint8_t addressWordBuff[3];

    addressWordBuff[0] = reg_addr;

    addressWordBuff[1] = uint8_t(data >> 8);
    addressWordBuff[2] = uint8_t(data);

    return HAL_I2C_Master_Transmit(hi2c_, INA_I2C_ADDR, addressWordBuff, 3, 1000);
}