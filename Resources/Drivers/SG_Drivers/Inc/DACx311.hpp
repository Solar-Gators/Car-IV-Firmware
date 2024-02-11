/*
 *  INA226.hpp
 *
 *  Created on: February 10, 2023
 *      Author: Matthew Shen
 * 
 * https://www.ti.com/lit/ds/symlink/dac5311.pdf?ts=1707525876004
 * 
 */

#ifndef DACx311_HPP_
#define DACx311_HPP_

#include "main.h"

#ifdef _cplusplus
extern "C" {
#endif

typedef enum {
    DACx311_MODE_NORMAL = 0x0U,
    DACx311_MODE_POWERDOWN_1K = 0x1U,
    DACx311_MODE_POWERDOWN_100K = 0x2U,
    DACx311_MODE_POWERDOWN_HIZ = 0x3U
} DACx311_ModeTypeDef;

class DACx311 {
public:
    DACx311(SPI_HandleTypeDef *phspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    HAL_StatusTypeDef SetValue(uint16_t value);
    HAL_StatusTypeDef Sleep();
private:
    SPI_HandleTypeDef *hspi_;
    GPIO_TypeDef *cs_port_;
    uint16_t cs_pin_;
    HAL_StatusTypeDef Write(DACx311_ModeTypeDef mode, uint16_t data);
};

#ifdef _cplusplus
}
#endif


#endif /* DACx311_HPP_ */