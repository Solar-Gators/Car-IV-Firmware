//Authors: Joshua Kwak,

#ifndef M24C02_HPP_
#define M24C02_HPP_

#include "stm32l4xx_hal.h"
#include <cstring>
#include <cstdint>
#include <vector>

using namespace std;

#define M24C02_I2C_ADDR (1010000 << 1)

class memory{ //Memory class containing ID, address, data type (i,f,c,b), and a name.

    
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

        char getType(){
            return type;
        }

        //Defines constructor
        //Supported data types: int (i), float (f), char (c)
        memory(int ID, uint8_t Addr, char type, char name[]){
            ID = ID;
            Addr = Addr;
            type = type;
            name = name;
            switch (type){

                case 'i':
                case 'f':
                size = 4;
                break;

                case 'c':
                case 'b':
                size = 1;
                break;

                default:
                size = 1;
                break;
            }


        }
        
        memory(){
            ID = -1;
            Addr = 0x00;
            type = 'n';
            name = "null";
            size = 0;

        }

    private:
        int ID;
        uint8_t Addr;
        uint8_t size;
        char* name;
        char type;
};

//Include all memory objects below in the array.

int test[] = {
    1,2,3,4
};

memory storage[] = { //Defines an array to store all of the memory objects in.
    memory(0,0x00, 'f', "Potential"),
    memory(1, 0x03, 'f', "Integral"),
    memory(2, 0x08, 'f', "Derivative"),
    memory(3, 0x0C, 'f', "Odometer"),
    memory(4, 0x10, 'f', "Regen"),
    memory(5, 0x14, 'f', "Speed")
};
//All address values are in hex.
//Floats and Ints are 4 bytes, Chars and Bools are 1 byte.



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
HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length);

#endif /* USER_HPP_ */



