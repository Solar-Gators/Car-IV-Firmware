/*
 *  BQ76952.hpp
 *
 *  Created on: October 20, 2023
 *      Author: Matthew Shen
 * 
 *  https://github.com/LibreSolar/bms-firmware/tree/main
 */

#ifndef BQ76952_HPP_
#define BQ76952_HPP_

#include "main.h"
#include "BQ769X2_Registers.h"
#include <math.h>

#define BQ_I2C_ADDR_WRITE 0x10
#define BQ_I2C_ADDR_READ 0x11

class BQ76952 {
public:
    HAL_StatusTypeDef Init(I2C_HandleTypeDef *hi2c);
    HAL_StatusTypeDef ConfigUpdateMode(bool config_update); // TODO
    HAL_StatusTypeDef ReadVoltages();       // TODO: Add support for connected cells
    HAL_StatusTypeDef ReadSafetyFaults();
    HAL_StatusTypeDef ReadCurrent();

    HAL_StatusTypeDef Shutdown();

    int16_t GetCellVoltage(uint32_t cell_num);
    int16_t GetPackVoltage();
    int16_t GetStackVoltage();
    int16_t GetAvgCellVoltage();
    int16_t GetHighCellVoltage();
    int16_t GetLowCellVoltage();
private:
    HAL_StatusTypeDef WriteBytes(const uint8_t reg_addr, const uint8_t *data, const size_t num_bytes);
    HAL_StatusTypeDef ReadBytes(uint8_t reg_addr, uint8_t *data, const size_t num_bytes);
    HAL_StatusTypeDef DirectReadU1(const uint8_t reg_addr, uint8_t *value);
    HAL_StatusTypeDef DirectReadI1(const uint8_t reg_addr, int8_t *value);
    HAL_StatusTypeDef DirectReadU2(const uint8_t reg_addr, uint16_t *value); // TODO
    HAL_StatusTypeDef DirectReadI2(const uint8_t reg_addr, int16_t *value); // TODO
    HAL_StatusTypeDef SubcmdRead(const uint16_t subcmd, uint32_t *value, const size_t num_bytes);
    HAL_StatusTypeDef SubcmdReadU1(const uint16_t subcmd, uint8_t *value);
    HAL_StatusTypeDef SubcmdReadU2(const uint16_t subcmd, uint16_t *value);
    HAL_StatusTypeDef SubcmdReadU4(const uint16_t subcmd, uint32_t *value);
    HAL_StatusTypeDef SubcmdReadI1(const uint16_t subcmd, int8_t *value);       // TODO: Remove
    HAL_StatusTypeDef SubcmdReadI2(const uint16_t subcmd, int16_t *value);     // TODO: Remove
    HAL_StatusTypeDef SubcmdReadI4(const uint16_t subcmd, int32_t *value);     // TODO: Remove
    HAL_StatusTypeDef SubcmdWrite(const uint16_t subcmd, const uint32_t value, const size_t num_bytes);
    HAL_StatusTypeDef SubcmdCmdOnly(const uint16_t subcmd);
    HAL_StatusTypeDef EnableThermistorPins(); // should get called by eventual init function

    I2C_HandleTypeDef *hi2c_;

    int16_t cell_voltages_[16];         // Voltage of each cell in mV
    int16_t avg_cell_voltage_;          // Average cell voltage in mV
    int16_t high_cell_voltage_;         // Highest cell voltage in mV
    int16_t low_cell_voltage_;          // Lowest cell voltage in mV
    int16_t pack_current_;

    int16_t pack_voltage_;              // Pack voltage in 10 mV
    int16_t stack_voltage_;             // Stack voltage in 10 mV
};

#endif  /* BQ76952_HPP_ */
