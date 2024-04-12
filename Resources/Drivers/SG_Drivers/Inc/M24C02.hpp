//Authors: Joshua Kwak,

#ifndef M24C02_HPP_
#define M24C02_HPP_

#include "stm32l4xx_hal.h"

using namespace std;


typedef member;
#define M24C02_I2C_ADDR (0b10100000)

class M24C02{

    public:

        //Constructor definitions.
        M24C02();
        M24C02(I2C_HandleTypeDef *i2cHandle);

        //High level general functions

        HAL_StatusTypeDef ReadAll(uint8_t *data);
        HAL_StatusTypeDef UpdateOne(member memb, uint8_t *newVal);
        void ShiftData(uint8_t startAddr, uint8_t byteShift); //Creates a space for new data to be added to either sub-struct of the memory struct. Pass the type (f,i,c,b) and the sub-struct (v,b).
        

        //High level individual functions
        //VCU
        HAL_StatusTypeDef ReadVCU(uint8_t *data);
        HAL_StatusTypeDef TickOdometer();

        //BMS
        HAL_StatusTypeDef ReadBMS(uint8_t *data);
    
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
//Removing padding ensures that only the bytes that are storing data re used.
#pragma pack (push, 1)

//Defines a struct to save the data. This struct will be converted to a byte pointer to be saved to the EEPROM.
//On the reciving end (GUI) this struct can be used to convert the bytes back to the struct, retaining the data and their locations.

//Order for all sub-structs is as follows.
//floats
//ints
//chars
//bools
//If new data is to be added, first run the AddDataTo function with the proper data type and substruct
//Then add the data to the end of the list for that type of data.
struct {

    float potential;
    float integral;
    float derivative;
    float odomter;
    float regen;
    float speed;

    int num;

    char test;

    bool fun;
    
} typedef VCU;

struct {

    float maxTemp;

} typedef BMS;

struct{

    //DO NOT CHANGE ORDER OF STRUCTS
    VCU VCU;
    BMS BMS;

} typedef memory;

#pragma pack(pop)

#endif /* USER_HPP_ */



