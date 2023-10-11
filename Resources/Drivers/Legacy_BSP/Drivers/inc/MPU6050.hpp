/*
 * MPU6050.hpp
 *
 *  Created on: Jan 24, 2022
 *      Author: John Carr
 *  Adapted from: https://github.com/leech001/MPU6050
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_MPU6050_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_MPU6050_HPP_

#include "main.h"
#include "stdint.h"

namespace SolarGators {
namespace Drivers {

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} point_3d_t;

typedef struct {
  float x;
  float y;
  float z;
} fpoint_3d_t;

class MPU6050 {
public:
  MPU6050(I2C_HandleTypeDef *hi2c, uint8_t addr);
  virtual ~MPU6050();
  uint8_t Init();
  void ReadAccel();
  void ReadGyro();
  void ReadTemp();
  void ReadAll();

  point_3d_t GetRawAccel();
  fpoint_3d_t GetAdjAccel();
  point_3d_t GetLastAccel();
  fpoint_3d_t GetLastAdjAccel();

  point_3d_t GetRawGyro();
  fpoint_3d_t GetAdjGyro();
  point_3d_t GetLastGyro();
  fpoint_3d_t GetLastAdjGyro();

  uint16_t GetTemp();
  uint16_t GetLastTemp();

  bool Update();

  static constexpr uint8_t WHO_AM_I_REG     = 0x75;
  static constexpr uint8_t PWR_MGMT_1_REG   = 0x6B;
  static constexpr uint8_t SMPLRT_DIV_REG   = 0x19;
  static constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
  static constexpr uint8_t ACCEL_XOUT_H_REG = 0x3B;
  static constexpr uint8_t TEMP_OUT_H_REG   = 0x41;
  static constexpr uint8_t GYRO_CONFIG_REG  = 0x1B;
  static constexpr uint8_t GYRO_XOUT_H_REG  = 0x43;
  static constexpr uint16_t i2c_timeout     = 100;

  // Setup MPU6050
  static constexpr uint8_t MPU6050_ADDR1     = 0xD0;
private:
  point_3d_t accel_;
  point_3d_t gyro_;
  uint16_t temperature_;
  I2C_HandleTypeDef *hi2c_;
  uint8_t addr_;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_MPU6050_HPP_ */
