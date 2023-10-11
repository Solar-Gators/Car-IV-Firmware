/*
 * LTC2630.cpp
 *
 *  Created on: Feb 28, 2022
 *      Author: John Carr
 */

#include "LTC2630.hpp"

namespace SolarGators {
namespace Drivers {

LTC2630::LTC2630(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, OperatingMode mode):
    hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin), mode_(mode), current_val_(0)
{ }

void LTC2630::Init()
{
  WriteAndUpdate(0x00);
}

LTC2630::~LTC2630()
{
  // If it falls out of scope we don't wana be stuck accelerating
  WriteAndUpdate(0x00);
}

void LTC2630::WriteData(uint16_t data)
{
  pending_val_ = data;
  data = AdjData(data);
  Write(C_WriteData, data);
}
void LTC2630::UpdateOutput()
{
  uint16_t data = 0x0000;
  Write(C_UpdateOutput, data);
  current_val_ = pending_val_;
}
void LTC2630::WriteAndUpdate(uint16_t data)
{
  current_val_ = data;
  data = AdjData(data);
  Write(C_WriteAndUpdate, data);
}
void LTC2630::PowerDown()
{
  uint16_t data = 0x0000;
  Write(C_PowerDown, data);
}
void LTC2630::SetRefInternal()
{
  uint16_t data = 0x0000;
  Write(C_SetRefInternal, data);
}
void LTC2630::SetRefVcc()
{
  uint16_t data = 0x0000;
  Write(C_SetRefVcc, data);
}

void LTC2630::Write(uint8_t command, uint16_t data)
{
  uint8_t buff[3]={ command << 4, data >> 8, data & 0xFF };
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi_, buff, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

uint16_t LTC2630::AdjData(uint16_t data)
{

  switch (mode_) {
    case OperatingMode::Bit8:
      return data << 8;
    case OperatingMode::Bit10:
      return data << 6;
    case OperatingMode::Bit12:
      return data << 4;
    default:
      return 0;  // Fail Safe
  }
}

} /* namespace Drivers */
} /* namespace SolarGators */
