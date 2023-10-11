/*
 * RFD900x.hpp
 *
 *  Created on: Jan 28, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_INC_RFD900X_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_INC_RFD900X_HPP_

#include <cmsis_os.h>
#include "main.h"
#include "etl/queue.h"
#include "etl/memory_model.h"

#include "DataModule.hpp"
#include "Radio.hpp"

namespace SolarGators {
namespace Drivers {

class RFD900x : public Radio {
public:
  RFD900x(UART_HandleTypeDef* huart);
  virtual ~RFD900x();
  void Init();
  void SendData(uint8_t* data, uint32_t size);
  void SendByte(uint8_t data);
private:
  UART_HandleTypeDef* huart_;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_INC_RFD900X_HPP_ */
