/*
 * LTC1661.hpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Matthew Shen
 */

#ifndef LTC1661_HPP_
#define LTC1661_HPP_

#include "main.h"

class LTC1661 {
public:
    LTC1661(SPI_HandleTypeDef *phspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    LTC1661(SPI_HandleTypeDef *phspi);
    HAL_StatusTypeDef Wake();
    HAL_StatusTypeDef Sleep();
    HAL_StatusTypeDef LoadA(uint16_t value);
    HAL_StatusTypeDef LoadB(uint16_t value);
    HAL_StatusTypeDef LoadAB(uint16_t value);
private:
    HAL_StatusTypeDef Write(uint8_t cmd, uint16_t data);
    SPI_HandleTypeDef *phspi_;
    GPIO_TypeDef *cs_port_ = 0;
    uint16_t cs_pin_ = 0;
};

#endif /* LTC1661_HPP_ */