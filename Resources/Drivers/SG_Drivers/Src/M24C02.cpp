//Authors: Joshua Kwak,


//High level functions
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

bool intToBytes(int val, uint8_t &bytes){

}

bool cstringToBytes(const char *name, int &length){


}

bool getData(memory obj, uint8_t *data){

}




bool fetchMem(uint8_t *info){
	try{
		int numStored = sizeof(storage);

		int currentSize = 0;
		int reqSize = 0;
		int strSize = 0;
		uint8_t tempInfo[20 * numStored];
		
		
	
		for(int i = 0 ; i < numStored ; i++){

			const char *tempName = storage[i].getName();
			bool pass = cstringToBytes(tempName, strSize);

			uint8_t tempInt;
			intToBytes(strSize, tempInt);

			tempInfo[currentSize] = tempInt;
			currentSize++;

			for(int j ; j < strSize ; j++){
				tempInfo[currentSize + j] = tempName[j];

			}
			currentSize += strSize;

			if (pass == false){
				throw 1;
			}

			//getData



		}

	
		delete tempInfo;
		//delete tempName;

	}

	catch(...){
	
	}



}



//Low level functions

HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data){ //Function to read data off the EEPROM.
    return HAL_I2C_Mem_Read(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data, int length){ //Function to read data off the EEPROM. Utilizes overloading.
    return HAL_I2C_Mem_Read(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data){ //Function to write data to the EEPROM.
	return HAL_I2C_Mem_Write(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data, int length){ //Function to write data to the EEPROM. Utilizes overloading.
	return HAL_I2C_Mem_Write(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}
