//Authors: Joshua Kwak,


//High level functions
#include "M24C02.hpp"

union FourBytes{
	float f;
	int i;
	uint8_t bytesArray[4];
};

union dataStorage{
	memory obj;
	uint8_t bytesArray[sizeof(memory)];
};

void FloatToBytes(float val, uint8_t* bytes){ //Converts float to a 4 byte array. Pass the float and an pointer to the save location of the bytes.

 	FourBytes temp;

 	temp.f = val;

 	for(int i = 0 ; i < 4; i++){
 		bytes[i] = temp.bytesArray[i];
 	}

}

void BytesToFloat(float &val, uint8_t* bytes){

	FourBytes temp;

 	for(int i = 0 ; i < 4; i++){
 		temp.bytesArray[i] = bytes[i];
 	}

 	val = temp.f;
}

void intToBytes(int val, uint8_t* bytes){

 	FourBytes temp;

	temp.i = val;

 	for(int i = 0 ; i < 4; i++){
 		bytes[i] = temp.bytesArray[i];
	}

}

void BytesToInt(int &val, uint8_t* bytes){

 	FourBytes temp;

 	for(int i = 0 ; i < 4; i++){
 		temp.bytesArray[i] = bytes[i];
 	}

 	val = temp.i;

}

void StructToBytes(memory obj, uint8_t* bytes){

	dataStorage temp;

	temp.obj = obj;

	for(int i = 0 ; i < sizeof(bytes) ; i++){
		bytes[i] = temp.bytesArray[i];
	}


}

void BytesToStruct(memory obj, uint8_t* bytes){

	dataStorage temp;

	for(int i = 0 ; i < sizeof(obj) ; i++){
		temp.bytesArray[i] = bytes[i];
	}

	obj = temp.obj;
}

memory storage;


M24C02::M24C02(){}

M24C02::M24C02(I2C_HandleTypeDef *i2cHandle){
	i2cHandle = i2cHandle;

}

HAL_StatusTypeDef M24C02::M24C02_ReadAll(uint8_t *data){ 

	try{
		HAL_StatusTypeDef HALStat = M24C02_ReadRegister(0x00, data, sizeof(storage));
		return HALStat;
	}
	catch(...){
		return HAL_ERROR;
	}

}

HAL_StatusTypeDef M24C02::M24C02_UpdateOne(int ID, uint8_t *newVal){

	memory tempStore;

	uint8_t *structBytes = new uint8_t;
	float tempFloat;
	int tempInt;
	

	try{

		M24C02_ReadRegister(0x00, structBytes, sizeof(storage));

		BytesToStruct(tempStore, structBytes);

		switch(ID){
			case 0:
				
				BytesToFloat(tempFloat, newVal);
				tempStore.potential = tempFloat; 
				break;



		}


		delete structBytes;

	} catch(...){

		delete structBytes;
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef M24C02::M24C02_TickOdometer(){

	uint8_t *structBytes = new uint8_t;
	memory tempStore;

	HAL_StatusTypeDef HALStat;
	try{
		HALStat = M24C02_ReadRegister(0x00, structBytes, sizeof(storage));

		if (HALStat == HAL_ERROR){
			throw;
		} else {
			BytesToStruct(tempStore, structBytes);
			tempStore.odomter += 1;
			StructToBytes(tempStore, structBytes);

			HALStat = M24C02_WriteRegister(0x00, structBytes, sizeof(storage));
			if (HALStat == HAL_ERROR){
				throw;
				
			} else {

				delete structBytes;
				return HAL_OK;
			}
		}

	}
	catch(...){

		delete structBytes;
		return HAL_ERROR;
	}

}

HAL_StatusTypeDef M24C02::M24C02_ReadRegister(uint8_t reg, uint8_t *data, int length = 1){
	return HAL_I2C_Mem_Read(this->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02::M24C02_WriteRegister(uint8_t reg, uint8_t *data, int length = 1){
	return HAL_I2C_Mem_Write(this->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}




//Low level functions

HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data, int length = 1){ //Function to read data off the EEPROM.
    return HAL_I2C_Mem_Read(dev->getI2CHandle(), M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length = 1){ //Function to write data to the EEPROM.
	return HAL_I2C_Mem_Write(dev->getI2CHandle(), M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}
