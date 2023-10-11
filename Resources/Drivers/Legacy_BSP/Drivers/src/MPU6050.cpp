/*
 * MPU6050.cpp
 *
 *  Created on: Jan 24, 2022
 *      Author: John Carr
 */

#include "MPU6050.hpp"

namespace SolarGators {
namespace Drivers {

MPU6050::MPU6050(I2C_HandleTypeDef *hi2c, uint8_t addr):hi2c_(hi2c),addr_(addr)
{ }

MPU6050::~MPU6050()
{ }

uint8_t MPU6050::Init()
{
  uint8_t check;
  uint8_t Data;

  // check device ID WHO_AM_I

  HAL_I2C_Mem_Read(hi2c_, addr_, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

  if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
  {
      // power management register 0X6B we should write all 0's to wake the sensor up
      Data = 0;
      HAL_I2C_Mem_Write(hi2c_, addr_, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

      // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
      Data = 0x07;
      HAL_I2C_Mem_Write(hi2c_, addr_, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

      // Set accelerometer configuration in ACCEL_CONFIG Register
      // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
      Data = 0x00;
      HAL_I2C_Mem_Write(hi2c_, addr_, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

      // Set Gyroscopic configuration in GYRO_CONFIG Register
      // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> 250 deg/s
      Data = 0x00;
      HAL_I2C_Mem_Write(hi2c_, addr_, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
      return 0;
  }
  return 1;
}
void MPU6050::ReadAccel()
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from ACCEL_XOUT_H register

  HAL_I2C_Mem_Read(hi2c_, addr_, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

  accel_.x = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
  accel_.y = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
  accel_.z = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
}
void MPU6050::ReadGyro()
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from GYRO_XOUT_H register

  HAL_I2C_Mem_Read(hi2c_, addr_, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

  gyro_.x = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
  gyro_.y = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
  gyro_.z = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
}
void MPU6050::ReadTemp()
{
  uint8_t Rec_Data[2];
  int16_t temp;

  // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

  HAL_I2C_Mem_Read(hi2c_, addr_, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

  temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
  temperature_ = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}
void MPU6050::ReadAll()
{
  uint8_t Rec_Data[14];

  // Read 14 BYTES of data starting from ACCEL_XOUT_H register

  HAL_I2C_Mem_Read(hi2c_, addr_, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

  accel_.x = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
  accel_.y = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
  accel_.z = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
  temperature_ = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
  gyro_.x = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
  gyro_.y = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
  gyro_.z = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);
}

point_3d_t MPU6050::GetRawAccel()
{
  ReadAccel();
  return GetLastAccel();
}
fpoint_3d_t MPU6050::GetAdjAccel()
{
  ReadAccel();
  return GetLastAdjAccel();
}
point_3d_t MPU6050::GetLastAccel()
{
  return accel_;
}
fpoint_3d_t MPU6050::GetLastAdjAccel()
{
  fpoint_3d_t ret;
  /*** convert the RAW values into acceleration in 'g'
       we have to divide according to the Full scale value set in FS_SEL
       I have configured FS_SEL = 0. So I am dividing by 16384.0
       for more details check ACCEL_CONFIG Register              ****/

  ret.x = static_cast<float>(accel_.x) / 16384.0;
  ret.y = static_cast<float>(accel_.y) / 16384.0;
  ret.z = static_cast<float>(accel_.z) / 14418.0;

  return ret;
}

point_3d_t MPU6050::GetRawGyro()
{
  ReadGyro();
  return gyro_;
}
fpoint_3d_t MPU6050::GetAdjGyro()
{
  ReadGyro();
  return GetLastAdjGyro();
}
point_3d_t MPU6050::GetLastGyro()
{
  return gyro_;
}

fpoint_3d_t MPU6050::GetLastAdjGyro()
{
  fpoint_3d_t ret;
  /*** convert the RAW values into dps (�/s)
       we have to divide according to the Full scale value set in FS_SEL
       I have configured FS_SEL = 0. So I am dividing by 131.0
       for more details check GYRO_CONFIG Register              ****/

  ret.x = static_cast<float>(gyro_.x) / 131.0;
  ret.y = static_cast<float>(gyro_.y) / 131.0;
  ret.z = static_cast<float>(gyro_.z) / 131.0;
  return ret;
}

} /* namespace Drivers */
} /* namespace SolarGators */
