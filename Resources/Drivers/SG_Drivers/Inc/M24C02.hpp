//Authors: Joshua Kwak,

#ifndef M24C02_HPP_
#define M24C02_HPP_

#include "stm32l4xx_hal.h"

using namespace std;



#define M24C02_I2C_ADDR (0b10100000)

class M24C02{

    public:

        //Constructor definitions.
        M24C02();
        M24C02(I2C_HandleTypeDef *i2cHandle);

        //High level functions

        HAL_StatusTypeDef M24C02_ReadAll(float *data);
        HAL_StatusTypeDef M24C02_UpdateOne(int ID, uint8_t newVal);
        HAL_StatusTypeDef M24C02_TickOdometer();
    
        //Low level functions

        HAL_StatusTypeDef M24C02_ReadRegister(uint8_t reg, uint8_t *data, int length = 1);

        HAL_StatusTypeDef M24C02_WriteRegister(uint8_t reg, uint8_t *data, int length = 1);

        //Getter Functions

        I2C_HandleTypeDef* getI2CHandle(){
            return i2cHandle;
        }

    private:
        I2C_HandleTypeDef* i2cHandle;




};

//Pragma pack prevents padding from being added to the struct. 
//With padding, additional bytes are added to align the values.
//Removing padding ensures that only the bytes that are storing data re used.
#pragma pack (push, 1)

//Defines a struct to save the data. This struct will be converted to a byte pointer to be saved to the EEPROM.
//On the reciving end (GUI) this struct can be used to convert the bytes back to the struct, retaining the data and their locations.
struct {

    float potential;
    float integral;
    float derivative;
    float odomter;
    float regen;
    float speed;
    bool fun;
    char test;
    int num;




} typedef memory;
#pragma pack(pop)

//High level functions





#endif /* USER_HPP_ */



