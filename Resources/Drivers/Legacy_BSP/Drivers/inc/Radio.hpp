/*
 * Radio.hpp
 *
 *  Created on: Feb 3, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_RADIO_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_RADIO_HPP_

#include "etl/callback.h"
#include "etl/function.h"
#include "main.h"

namespace SolarGators {
namespace Drivers {

class Radio {
public:
  Radio();
  virtual ~Radio();
  virtual void SendData(uint8_t* buff, uint32_t size) = 0;
  virtual void SendByte(uint8_t data) = 0;
  virtual void Init() = 0;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_RADIO_HPP_ */
