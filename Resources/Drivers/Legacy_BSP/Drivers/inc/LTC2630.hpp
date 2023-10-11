/*
 * LTC2630.hpp
 *
 *  Created on: Feb 28, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_LTC2630_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_LTC2630_HPP_

#include "stm32f4xx_hal.h"

namespace SolarGators {
namespace Drivers {


typedef enum class OperatingMode : uint8_t{
  Bit8 = 0,
  Bit10 = 1,
  Bit12 = 2
} OperatingMode;

class LTC2630 {
  SPI_HandleTypeDef* hspi_;
  GPIO_TypeDef* cs_port_;
  uint16_t cs_pin_;
  uint16_t current_val_;
  uint16_t pending_val_;
  OperatingMode mode_;

  static constexpr uint8_t C_WriteData      = 0x00;
  static constexpr uint8_t C_UpdateOutput   = 0x01;
  static constexpr uint8_t C_WriteAndUpdate = 0x03;
  static constexpr uint8_t C_PowerDown      = 0x04;
  static constexpr uint8_t C_SetRefInternal = 0x06;
  static constexpr uint8_t C_SetRefVcc      = 0x07;

  uint16_t AdjData(uint16_t data);
  void Write(uint8_t command, uint16_t data);
public:

  LTC2630(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, OperatingMode mode);
  void Init();
  virtual ~LTC2630();
  void WriteData(uint16_t data);
  void UpdateOutput();
  void WriteAndUpdate(uint16_t data);
  void PowerDown();
  void SetRefInternal();
  void SetRefVcc();
  uint16_t GetCurrentVal() const;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_LTC2630_HPP_ */
