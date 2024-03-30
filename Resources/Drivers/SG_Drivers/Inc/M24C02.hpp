//Authors: Joshua Kwak,

#ifndef M24C02_HPP_
#define M24C02_HPP_

#include "stm32l4xx_hal.h"

using namespace std;



#define M24C02_I2C_ADDR (0b10100000)


class M24C02{



}

struct {



} typedef memory;

//High level functions
HAL_StatusTypeDef M24C02_INIT(M24C02 *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef M24C02_ReadAll(M24C02 *dev, float *data);
HAL_StatusTypeDef M24C02_UpdateOne(M24C02 *dev, int ID, uint8_t newVal);
HAL_StatusTypeDef M24C02_TickOdometer(M24C02 *dev);

//Low level functions

HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data, int length = 1);

HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data, int length = 1);

#endif /* USER_HPP_ */



