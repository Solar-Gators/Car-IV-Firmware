/*
 * LED.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: John Carr
 */

#include <LED.hpp>

namespace SolarGators::Drivers {

LED::LED(const char* name, GPIO_TypeDef* port, uint16_t pin):on_(false), name_(name), port_(port), pin_(pin)
{
  TurnOff();
}

LED::~LED()
{

}

void LED::TurnOn()
{
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
  on_ = true;
}
void LED::TurnOff()
{
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
  on_ = false;
}
void LED::Toggle()
{
  HAL_GPIO_TogglePin(port_, pin_);
  on_ = !on_;
}
void LED::UpdateLed()
{
  if(on_)
    TurnOn();
  else
    TurnOff();
}

bool LED::GetStatus()
{
  return on_;
}

bool LED::IsOn()
{
  return on_;
}

bool LED::IsOff()
{
  return !on_;
}

} /* namespace SolarGators */
