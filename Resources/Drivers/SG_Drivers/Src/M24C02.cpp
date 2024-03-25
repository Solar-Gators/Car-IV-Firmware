//Authors: Joshua Kwak,

#include "M24C02.hpp"

void Float_To_Bytes(float val, uint8_t* bytes){ //Converts float to a 4 byte array. Pass the float and an pointer to the save location of the bytes.

	union U //Creates a shared memory space of the largest item (4 bytes).
	{
		float tempFloat; //Both items are saved in the same memory space concurrently.
		uint8_t bytesArray[4];
	};

	//U u = val;

	//memcpy(bytes, u.tempFloat, 4); //Copies the data from the float value to the bytes
}





//Low level functions

HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Read(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data, int length){
    return HAL_I2C_Mem_Read(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}


