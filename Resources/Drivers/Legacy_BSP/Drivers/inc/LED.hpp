/*
 * LED.h
 *
 *  Created on: Oct 27, 2021
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DRIVERS_INC_LED_HPP_
#define SOLARGATORSBSP_DRIVERS_INC_LED_HPP_

#include "main.h"

namespace SolarGators::Drivers {

class LED {
public:
  LED(const char* name, GPIO_TypeDef* port, uint16_t pin);
  virtual ~LED();
  void TurnOn();
  void TurnOff();
  void Toggle();
  void UpdateLed();
  bool GetStatus();
  bool IsOn();
  bool IsOff();
private:
  bool on_;
  const char* name_;
  GPIO_TypeDef* port_;
  uint16_t pin_;
};

} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DRIVERS_INC_LED_HPP_ */
