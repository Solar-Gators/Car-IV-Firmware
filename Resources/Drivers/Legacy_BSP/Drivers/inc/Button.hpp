/*
 * button.hpp
 *
 *  Created on: Sep 19, 2021
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DRIVERS_INC_BUTTON_HPP_
#define SOLARGATORSBSP_DRIVERS_INC_BUTTON_HPP_

#include "etl/delegate.h"

#include "cmsis_os.h"
#include "main.h"

namespace SolarGators {
namespace Drivers {
class Button
{
public:
  Button(const char* name, GPIO_TypeDef* port, uint16_t pin, uint16_t press_time = 0, GPIO_PinState active_state = GPIO_PIN_RESET);
  ~Button();
  GPIO_PinState ReadPin();
  void HandlePress();
  void Debounce();
  etl::delegate<void(void)> action_;
  // Name of the function
  const char* name_;
  GPIO_TypeDef* port_;
  uint16_t pin_;
  uint8_t action_called_time_;
private:
  // Time that the button needs to be pressed for in order to register (Milliseconds)
  const uint16_t press_time_;
  GPIO_PinState active_state_;
  uint32_t press_count_;
  uint32_t last_valid_press_time_;
  // Debounce Time in ticks (which is milliseconds in our case)
  const uint16_t debounce_time_ = 50;
  // If the button is able to generate an action (Used for debouncing)
  bool disabled_;
};
} /* namespcae Drivers */
} /* namespace SolarGators */


#endif /* SOLARGATORSBSP_DRIVERS_INC_BUTTON_HPP_ */
