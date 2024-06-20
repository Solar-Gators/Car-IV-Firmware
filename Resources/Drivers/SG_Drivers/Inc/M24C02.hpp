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

        //High level general functions

    
        HAL_StatusTypeDef ReadAll(uint8_t *data);
        HAL_StatusTypeDef UpdateAll(void* struc);
        void ShiftDataForwards(uint8_t startAddr, uint8_t byteShift); //Creates a space for new data to be added to either sub-struct of the memory struct. Pass the type (f,i,c,b) and the sub-struct (v,b).
        void ShiftDataBackwards(uint8_t startAddr, uint8_t byteshift);
        

        //High level individual functions
        //VCU
        HAL_StatusTypeDef ReadVCU(uint8_t *data);
        HAL_StatusTypeDef TickOdometer();
        

        /*
        Update functions should be defined as follows
        HAL_StatusTypeDef Change{name of member}({data type} val)
        */

        //Update Functions
        HAL_StatusTypeDef ChangePotential(float val);
        HAL_StatusTypeDef ChangeIntegral(float val);
        HAL_StatusTypeDef ChangeDerivative(float val);
        HAL_StatusTypeDef ChangeOdometer(float val);
        HAL_StatusTypeDef ChangeRegen(float val);
        HAL_StatusTypeDef ChangeSpeed(float val);

        //BMS
        HAL_StatusTypeDef ReadBMS(uint8_t *data);

        //Update Functions
        HAL_StatusTypeDef ChangeMaxTemp(float val);
    
        //Low level functions

        HAL_StatusTypeDef ReadRegister(uint8_t reg, uint8_t *data, int length = 1);

        HAL_StatusTypeDef WriteRegister(uint8_t reg, uint8_t *data, int length = 1);

        //Getter Functions

        I2C_HandleTypeDef* getI2CHandle(){
            return i2cHandle;
        }

    private:
        I2C_HandleTypeDef* i2cHandle;

};

//Pragma pack prevents padding from being added to the struct. 
//With padding, additional bytes are added to align the values.
//Removing padding ensures that only the bytes that are storing data are used.
#pragma pack (push, 1)

//Defines a struct to save the data. This struct will be converted to a byte pointer to be saved to the EEPROM.
//On the reciving end (GUI) this struct can be used to convert the bytes back to the struct, retaining the data and their locations.

//Order for all sub-structs is as follows.
//floats
//ints
//chars
//bools

//If new data is to be added, first run the ShiftData function with the proper memory address and substruct
//Then add the data to the end of the list for the desired type of data.
struct { //VCU Data

    float potential;
    float integral;
    float derivative;
    float odomter;
    float regen;
    float speed;

    //int integer;

    //char character;

    //bool boolean;
    
} typedef subVCU;

struct { //BMS Data

    float maxTemp;

} typedef subBMS;

struct{

    //DO NOT CHANGE ORDER OF STRUCTS
    subVCU VCU;
    subBMS BMS;

} typedef memory;

#pragma pack(pop)

#endif /* USER_HPP_ */



