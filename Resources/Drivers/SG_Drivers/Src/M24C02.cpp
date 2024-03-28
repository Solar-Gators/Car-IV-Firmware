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

bool cstringToBytes(const char *name, int &length, uint8_t *nameData){


}

bool getData(memory obj, uint8_t *data){

}




bool fetchInfo(uint8_t *info){
	try{
		int numStored = sizeof(storage);

		int currentSize = 0;
		int reqSize = 0;
		int strSize = 0;
		uint8_t tempInfo[20 * numStored];
		
		
	
		//Structure for data packets. byte 0, length of string in int, btyes n-m characters for the c-string, byte m+1 ID of object. Pattern repeats for all objects in storage.
		for(int i = 0 ; i < numStored ; i++){

			const char *tempName = storage[i].getName();
			uint8_t *nameData;
			bool pass = cstringToBytes(tempName, strSize, nameData);
			if (pass == false){
				throw;
			}


			tempInfo[currentSize] = strSize;
			currentSize++;

			for(int j = 0 ; j < strSize ; j++){

				tempInfo[currentSize + j] = nameData[j];

			}
			currentSize += strSize;

			delete nameData;

			tempInfo[currentSize] = storage[i].getID();
			currentSize++;

			delete tempName;

		}

		reqSize = currentSize;
		info = tempInfo;
		
		delete tempInfo;

	}

	catch(...){
		//strcpy((char*)buf, "Error: Conversion Error")
	
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
