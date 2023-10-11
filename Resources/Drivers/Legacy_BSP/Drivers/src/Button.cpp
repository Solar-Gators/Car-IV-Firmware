/*
 * Button.cpp
 *
 *  Created on: Sep 19, 2021
 *      Author: John Carr
 */

#include "Button.hpp"

#include "map"
namespace SolarGators::Drivers {

Button::Button(const char* name, GPIO_TypeDef* port, uint16_t pin, uint16_t press_time, GPIO_PinState active_state):
    name_(name),port_(port),pin_(pin),press_time_(press_time), active_state_(active_state), press_count_(0), last_valid_press_time_(0)
{
  disabled_ = false;
}

Button::~Button()
{
}

GPIO_PinState Button::ReadPin()
{
  return HAL_GPIO_ReadPin(port_, pin_);
}

void Button::HandlePress()
{
  if(last_valid_press_time_ + debounce_time_ < osKernelGetTickCount())
  {
    press_count_++;
    last_valid_press_time_ = osKernelGetTickCount();
    // Sleep until we should call action
    osDelay(debounce_time_ + press_time_);
    if(ReadPin() != active_state_)
      return;
    // Make sure button is still pressed
    // and there is an action assigned to it
    if(action_)
    {
      action_();
      action_called_time_++;
    }
  }
}

void Button::Debounce()
{
  disabled_ = false;
}

}
