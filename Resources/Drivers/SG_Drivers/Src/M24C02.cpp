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

HAL_StatusTypeDef M24C02::ReadAll(uint8_t *data){ 

	try{
		HAL_StatusTypeDef HALStat = ReadRegister(0x00, data, sizeof(storage));
		return HALStat;
	}
	catch(...){
		return HAL_ERROR;
	}

}

HAL_StatusTypeDef M24C02::UpdateOne(uint8_t *newVal){

	memory tempStore;

	uint8_t *structBytes = new uint8_t;
	float tempFloat;
	int tempInt;

	try{

		ReadRegister(0x00, structBytes, sizeof(storage));

		BytesToStruct(tempStore, structBytes);

		delete structBytes;

	} catch(...){

		delete structBytes;
		return HAL_ERROR;
	}
}

void M24C02::ShiftDataForwards(uint8_t startAddr, uint8_t byteShift){ //Shifts all the data starting at a given address for the given byte shift length. Used to make space to add new member variables in the structs.

	uint8_t *tempData = new uint8_t;

	// Read from start address until end of memory
	HAL_StatusTypeDef HALStat = ReadRegister(startAddr, tempData, (sizeof(memory) - startAddr));

	// Makes room for the data of size byteShift
	HAL_StatusTypeDef HALStat = WriteRegister(startAddr, 0x00, byteShift);
	// Write the rest of the data after the byteShift
	HAL_StatusTypeDef HALStat = WriteRegister(startAddr + byteShift, tempData, sizeof(tempData));

	delete tempData;
}

void M24C02::ShiftDataBackwards(uint8_t startAddr, uint8_t byteShift){

	uint8_t *tempData = new uint8_t;

	// Read from after what we want to shift till the end
	HAL_StatusTypeDef HALStat = ReadRegister(startAddr + byteShift, tempData, (sizeof(memory) - startAddr - byteShift));

	// Write starting at the start address, overwriting the memory at the byteShift
	HAL_StatusTypeDef HALStat = WriteRegister(startAddr, tempData, sizeof(tempData));
	// Write zeros at the end where memory would be
	HAL_StatusTypeDef HALStat = WriteRegister(startAddr + sizeof(tempData), 0x00, byteShift);

	delete tempData;

}

//VCU Functions

HAL_StatusTypeDef M24C02::TickOdometer(){

	uint8_t *structBytes = new uint8_t;
	memory tempStore;

	HAL_StatusTypeDef HALStat;
	try{
		HALStat = ReadRegister(0x00, structBytes, sizeof(storage));

		if (HALStat == HAL_ERROR){
			throw;
		} else {
			BytesToStruct(tempStore, structBytes);
			tempStore.VCU.odomter += 1;
			StructToBytes(tempStore, structBytes);

			HALStat = WriteRegister(0x00, structBytes, sizeof(storage));
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

HAL_StatusTypeDef M24C02::ReadVCU(uint8_t *data){

	uint8_t VCUAddr = 0x00;

	try{
		HAL_StatusTypeDef HALStat = ReadRegister(VCUAddr, data, sizeof(storage.VCU));
		return HALStat;
	}
	catch(...){
		return HAL_ERROR;
	}
}

//Update Functions
HAL_StatusTypeDef M24C02::ChangePotential(float val){
	
	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	}else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.VCU.potential = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}
	
}

HAL_StatusTypeDef M24C02::ChangeIntegral(float val){

	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	} else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.VCU.integral = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}

}

HAL_StatusTypeDef M24C02::ChangeDerivative(float val){

	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	}else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.VCU.derivative = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}

}

HAL_StatusTypeDef M24C02::ChangeOdometer(float val){

	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	}else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.VCU.odomter = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}

}

HAL_StatusTypeDef M24C02::ChangeRegen(float val){

	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	}else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.VCU.regen = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}

}

HAL_StatusTypeDef M24C02::ChangeSpeed(float val){

	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	}else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.VCU.speed = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}

}

//BMS Functions

HAL_StatusTypeDef M24C02::ReadBMS(uint8_t *data){

	uint8_t BMSAddr = 0x00 + sizeof(storage.VCU) - 0x01;

	try{
		HAL_StatusTypeDef HALStat = ReadRegister(BMSAddr, data, sizeof(storage.BMS));
		return HALStat;
	}
	catch(...){
		return HAL_ERROR;
	}
}

//Update Functions
HAL_StatusTypeDef M24C02::ChangeMaxTemp(float val){

	uint8_t *tempData = new uint8_t;
	HAL_StatusTypeDef HALStat;
	memory tempStruct;

	HALStat = ReadRegister(0x00, tempData, sizeof(memory));
	if(HALStat == ERROR){
		delete tempData;
		return HALStat;
	}else {

		BytesToStruct(tempStruct, tempData);
		tempStruct.BMS.maxTemp = val;
		StructToBytes(tempStruct, tempData);

		HALStat = WriteRegister(0x00, tempData, sizeof(memory));
		delete tempData;
		return HALStat;
	}

}

//Low Level Functions

HAL_StatusTypeDef M24C02::ReadRegister(uint8_t reg, uint8_t *data, int length = 1){
	return HAL_I2C_Mem_Read(this->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02::WriteRegister(uint8_t reg, uint8_t *data, int length = 1){
	return HAL_I2C_Mem_Write(this->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

