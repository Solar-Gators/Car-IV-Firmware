//Authors: Joshua Kwak,


//High level functions
#include "M24C02.hpp"

void Float_To_Bytes(float val, uint8_t* bytes){ //Converts float to a 4 byte array. Pass the float and an pointer to the save location of the bytes.

	union U //Creates a shared memory space of the largest item (4 bytes).
	{
		float tempFloat; //Both items are saved in the same memory space concurrently.
		uint8_t bytesArray[4];
	};

	U temp;

	temp.tempFloat = val;

	// Creates a temp variable to reassign the pointer
	uint8_t* temp = bytes;

	for(int i = 0 ; i < 4; i++){
		bytes[i] = temp.bytesArray[i];
	}

	delete temp;
}

void BytesToFloat(float &val, uint8_t* bytes){

	union U //Creates a shared memory space of the largest item (4 bytes).
	{
		float tempFloat; //Both items are saved in the same memory space concurrently.
		uint8_t bytesArray[4];
	};

	U temp;

	for(int i = 0 ; i < 4; i++){
		temp.bytesArray[i] = bytes[i];
	}

	val = temp.tempFloat;
}

bool intToBytes(int val, uint8_t* bytes){

	union U //Creates a shared memory space of the largest item (4 bytes).
	{
		int tempInt; //Both items are saved in the same memory space concurrently.
		uint8_t bytesArray[4];
	};

	U temp;

	temp.tempInt = val;

	uint8_t* tempBytes = bytes;

	for(int i = 0 ; i < 4; i++){
		bytes[i] = temp.bytesArray[i];
	}

	delete tempBytes;

}

void BytesToInt(int &val, uint8_t* bytes){

	union U //Creates a shared memory space of the largest item (4 bytes).
	{
		int tempInt; //Both items are saved in the same memory space concurrently.
		uint8_t bytesArray[4];
	};

	U temp;

	for(int i = 0 ; i < 4; i++){
		temp.bytesArray[i] = bytes[i];
	}

	val = temp.tempInt;

}

bool cstringToBytes(const char *name, int &length, uint8_t *nameData){
	
	int strLen = sizeof(name);
	length = strLen;

	uint8_t *temp = nameData;
		
		for(int i = 0 ; i < strLen ; i++){

			nameData[i] = name[i];

		}
		return true;
	}
	catch(...){
		return false;
	}

	delete temp;

}

// returns the data from the memory chip into data
HAL_StatusTypeDef getData(memory obj, uint8_t *data){

	uint8_t *temp = data;

	Hal_StatusTypeDef status = M24C02_ReadRegister(M24C02 *dev, obj.getAddr(), uint8_t *data, obj.getSize());

	delete temp;

	return status;
	
}

HAL_StatusTypeDef setData(memory obj, uint8_t *data){

	uint8_t *temp = data;
	HAL_StatusTypeDef status = M24C02_WriteRegister(M24C02 *dev, obj.getAddr(), uint8_t *data, obj.getSize());

	delete temp;

	return status;

}


HAL_StatusTypeDef M24C02FetchMemInfo(uint8_t *info){
	
}

//HAL_StatusTypeDef M24C02ReadAll{



//}

HAL_StatusTypeDef M24C02TickOdometer(memory odomObj){

	uint8_t tempBytes[4];
	getData[odomObj, tempBytes];


	int tempInt;
	BytesToInt(tempInt, tempBytes);

	tempInt += 1;

	IntToBytes(tempInt, tempBytes);



	

	

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
