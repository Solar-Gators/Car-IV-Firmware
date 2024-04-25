/*
 *  INA226.hpp
 *
 *  Created on: December 7, 2023
 *      Author: Jack Werner
 * 
 *  https://github.com/mongoose-os-libs/ina226-i2c/tree/master
 */

#ifndef INA226_HPP_
#define INA226_HPP_

#include "main.h"
#include <math.h>

#define INA_I2C_ADDR_WRITE 0b10001000
#define INA_I2C_ADDR_READ 0b10001001

#define INA_CONFIG 0x00
#define INA_SHUNT_VOLTAGE 0x01
#define INA_BUS_VOLTAGE 0x02
#define INA_POWER 0x03
#define INA_CURRENT 0x04
#define INA_CALIBRATION 0x05
#define INA_MASK_ENABLE 0x06
#define INA_ALERT_LIMIT 0x07
#define INA_MAN_ID 0xFE
#define INA_DIE_ID 0xFF

class INA226{
public:
    HAL_StatusTypeDef Init(I2C_HandleTypeDef *hi2c);
    HAL_StatusTypeDef GetShuntVoltage();
    HAL_StatusTypeDef GetCurrent();
    HAL_StatusTypeDef GetBusVoltage();
    HAL_StatusTypeDef GetPower();
    HAL_StatusTypeDef SetCalibrationReg(float ohms, float maxExpectedCurrent);
    HAL_StatusTypeDef Reset();
    HAL_StatusTypeDef SetConfig();

    float Current();
    float ShuntResistance();
    float BusVoltage();
    float ShuntVoltage();
private:
    HAL_StatusTypeDef ReadWordReg(uint8_t reg_addr, uint8_t *data);
    HAL_StatusTypeDef WriteWordReg(uint8_t reg_addr, uint16_t data);

    I2C_HandleTypeDef *hi2c_;
    float shunt_resistance_; // ohms
    float current_; // amps
    float bus_voltage_; // volts
    float shunt_voltage_; // volts
    float power_; // watts
    float current_LSB_; // Multiply by current register to get current in A
    float power_LSB_; // Multiply by power register to get power in W (always 25x current)
};


#endif 