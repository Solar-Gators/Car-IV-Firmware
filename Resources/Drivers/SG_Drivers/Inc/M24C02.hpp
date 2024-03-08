//Authors: Joshua Kwak,

#ifndef M24C02_HPP_
#define M24C02_HPP_

#include "stm32l4xx_hal.h"
#include "string.h"
#include <cstdint>
#include <vector>

using namespace std;

#define M2402_I2C_ADDR (1010000 << 1)

class memory{ //Memory class containing ID, address, size (in hex), and a name.

    public:

        //Defines getter functions
        int getID(){
            return ID;
        }

        uint8_t getAddr(){
            return Addr;
        }

        uint8_t getSize(){
            return size;
        }

        char* getName(){
            return name;
        }

        //Defines constructor
        memory(int ID, uint8_t Addr, uint8_t size, char name[]){
            ID = ID;
            Addr = Addr;
            size = size;
            name = name;
        }

    private:
        int ID;
        uint8_t Addr;
        uint8_t size;
        char name[];
};

/*Define a list of all of the names in order of assignment.
The order does not matter as assignment and calling is dynamic.*/
char* name[] = {"Potential", "Integral", "Derivative", "Odometer", "Regen", "Speed"};

typedef struct {
    I2C_HandleTypeDef *i2cHandle;
} M24C02;

//High level functions
HAL_StatusTypeDef M24C02_INIT(M24C02 *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef M24C02_ReadAll(M24C02 *dev, float *data);
HAL_StatusTypeDef M24C02_UpdateOne(M24C02 *dev, int ID, uint8_t newVal);
HAL_StatusTypeDef M24C02_TickOdometer(M24C02 *dev);

//Low level functions
//Utilize overloading so only one function is needed
HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length);

#endif /* USER_HPP_ */



