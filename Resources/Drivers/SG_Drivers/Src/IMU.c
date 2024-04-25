//IMU I2C driver
//Authors: Samuel Breslin, Kathrine Gonzalez
//11/20/2023

#include "IMU.h"

uint8_t IMU_INIT(IMU *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle    = i2cHandle;

	dev->accelData[0] = 0.0f;
	dev->accelData[1] = 0.0f;
	dev->accelData[2] = 0.0f;

	dev->gyroData[0] = 0.0f;
	dev->gyroData[1] = 0.0f;
	dev->gyroData[2] = 0.0f;


	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;
	status = IMU_ReadRegister(dev, WHO_AM_I_REG, &regData);
	errNum += (status != HAL_OK);
	//if(WHO_AM_I_REG != 0x68){
	//	return(255);
	//}

	regData = 0x00;
	status = IMU_WriteRegister(dev, PWR_WAKE_UP, &regData);
	errNum += (status != HAL_OK);

	regData = 0x00; //0x00 -> 250d/s, 0x08 -> 500d/s , 0x10->1000d/s, 0x18->2000d/s
	status = IMU_WriteRegister(dev, GYRO_CONFIG, &regData);
	errNum += (status != HAL_OK);

	regData = 0x08; //0x00 -> 2g, 0x08 -> 4g , 0x10->8g, 0x18->16g
	status = IMU_WriteRegister(dev, ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);

	regData = 0x01; //0x00 -> 2g, 0x08 -> 4g , 0x10->8g, 0x18->16g
	status = IMU_WriteRegister(dev, INT_CONFIG, &regData);
	errNum += (status != HAL_OK);

	return(errNum);
}

HAL_StatusTypeDef IMU_ReadAccel(IMU *dev){

	HAL_StatusTypeDef status = IMU_ReadRegisters(dev, ACCEL_XOUT_H , dev->accelRaw, 6);
	int16_t combinedData1 = ((uint16_t)dev->accelRaw[0] << 8) | dev->accelRaw[1];
	int16_t combinedData2 = ((uint16_t)dev->accelRaw[2] << 8) | dev->accelRaw[3];
	int16_t combinedData3 = ((uint16_t)dev->accelRaw[4] << 8) | dev->accelRaw[5];
	dev->accelData[0] = (float)combinedData1*(4.0/65536);
	dev->accelData[1] = (float)combinedData2*(4.0/65536);
	dev->accelData[2] = (float)combinedData3*(4.0/65536);

}
HAL_StatusTypeDef IMU_ReadGyro(IMU *dev){


	HAL_StatusTypeDef status = IMU_ReadRegisters(dev, GYRO_XOUT_H , dev->gyroRaw, 6);
	int16_t combinedData1 = ((int16_t)dev->gyroRaw[0] << 8) | dev->gyroRaw[1];
	int16_t combinedData2 = ((int16_t)dev->gyroRaw[2] << 8) | dev->gyroRaw[3];
	int16_t combinedData3 = ((int16_t)dev->gyroRaw[4] << 8) | dev->gyroRaw[5];
	dev->gyroData[0] = (float)combinedData1*(250.0/65536);
	dev->gyroData[1] = (float)combinedData2*(250.0/65536);
	dev->gyroData[2] = (float)combinedData3*(250.0/65536);

}
HAL_StatusTypeDef IMU_ReadTemp(IMU *dev){
	uint8_t rawData[2];

	HAL_StatusTypeDef status = IMU_ReadRegisters(dev, TEMP_OUT_H, rawData, 2);

	int16_t combinedData = ((int16_t)rawData[0] << 8) | rawData[1];
	dev->temp = ((float)combinedData /340.0) + 36.53;

	return(status);
}


//low level functions
HAL_StatusTypeDef IMU_ReadRegister(IMU *dev, uint8_t reg, uint8_t *data){
	return(HAL_I2C_Mem_Read(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY));
}

HAL_StatusTypeDef IMU_ReadRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length){
	return(HAL_I2C_Mem_Read(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));
}

HAL_StatusTypeDef IMU_WriteRegister(IMU *dev, uint8_t reg, uint8_t *data){
	HAL_I2C_Mem_Write(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
