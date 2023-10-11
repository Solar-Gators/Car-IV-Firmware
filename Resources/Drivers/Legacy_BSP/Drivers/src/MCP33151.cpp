/*
 * MCP33151.cpp
 *
 *  Created on: Jun 13, 2022
 *      Author: John Carr
 */

#include <MCP33151.hpp>
#include "cmsis_os.h"

namespace SolarGators {
namespace Drivers {

MCP33151::MCP33151(GPIO_TypeDef* cs_port, uint16_t cs_pin, SPI_HandleTypeDef* spi):
    cs_port_(cs_port),
    cs_pin_(cs_pin),
    spi_(spi)
{ }

MCP33151::~MCP33151()
{ }

void MCP33151::Init()
{
  // Should be calibrated during power on if we get weird readings then we'll need to adjust this
//  Calibrate();
  // Set CS low (oddly this is the idle state)
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

uint16_t MCP33151::Read()
{
  uint16_t data;
  // Start conversion
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
  // This is unfortunately blocking delay 2ms (t_cnv)
  osDelay(2);
  // Write Pin low (this will start output)
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
  HAL_SPI_Receive(spi_, (uint8_t*)&data, 2, HAL_MAX_DELAY);
//  data = (data) & 0x3FFF;
  // Keep pin low SDO with go high Z after 14 clk cycles
//  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
  return data;
}

void MCP33151::Calibrate()
{
  uint8_t buff[] = { 0xFF };
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
  // TODO: How many times?
  for (int i = 0; i < 1024/8; ++i)
  {
    HAL_SPI_Transmit(spi_, buff, 1, HAL_MAX_DELAY);
  }
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

} /* namespace Drivers */
} /* namespace SolarGators */
