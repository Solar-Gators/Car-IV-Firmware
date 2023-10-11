/*
 * MCP33151.hpp
 *
 *  Created on: Jun 13, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_INC_MCP33151_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_INC_MCP33151_HPP_

#include "main.h"

namespace SolarGators {
namespace Drivers {

class MCP33151 {
public:
  MCP33151(GPIO_TypeDef* cs_port, uint16_t cs_pin, SPI_HandleTypeDef* spi);
  virtual ~MCP33151();
  void Init();
  uint16_t Read();
  void Calibrate();
private:
  GPIO_TypeDef* cs_port_;
  uint16_t cs_pin_;
  SPI_HandleTypeDef* spi_;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_INC_MCP33151_HPP_ */
