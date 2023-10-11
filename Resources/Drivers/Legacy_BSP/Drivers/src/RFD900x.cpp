/*
 * RFD900x.cpp
 *
 *  Created on: Jan 28, 2022
 *      Author: John Carr
 */

#include <RFD900x.hpp>
#include <CAN.hpp>

namespace SolarGators {
namespace Drivers {

RFD900x::RFD900x(UART_HandleTypeDef* huart):Radio(),huart_(huart)
{ }

RFD900x::~RFD900x()
{ }

void RFD900x::Init()
{ }

inline void RFD900x::SendData(uint8_t* data, uint32_t size)
{
  for (uint32_t i = 0; i < size; ++i)
  {
    SendByte(data[i]);
  }
}

inline void RFD900x::SendByte(uint8_t data)
{
  while(!(this->huart_->Instance->ISR & USART_ISR_TXE));
  this->huart_->Instance->TDR = data;
}

} /* namespace Drivers */
} /* namespace SolarGators */
