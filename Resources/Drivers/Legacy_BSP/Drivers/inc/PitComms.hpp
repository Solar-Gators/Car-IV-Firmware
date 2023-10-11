/*
 * PitComms.hpp
 *
 *  Created on: Feb 3, 2022
 *      Author: John Carr
 *  Description: This is the abstraction class for communicating with the pit via a serial device.
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_PITCOMMS_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_PITCOMMS_HPP_

#include "etl/queue.h"
#include "etl/iterator.h"

#include "DataModule.hpp"
#include "Radio.hpp"

namespace SolarGators {
namespace Drivers {

class PitComms {
private:
  static constexpr uint8_t MAX_PACKETS = 10;
  static constexpr uint8_t START_CHAR = 0xFF;
  static constexpr uint8_t ESC_CHAR = 0x2F;
  static constexpr uint8_t END_CHAR = 0x3F;
  SolarGators::Drivers::Radio* radio_;
public:
  PitComms(SolarGators::Drivers::Radio* radio);
  virtual ~PitComms();
  void Init();
  void SendDataModule(SolarGators::DataModules::DataModule& data_module);
  uint8_t EscapeData(uint8_t data);
private:
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_PITCOMMS_HPP_ */
