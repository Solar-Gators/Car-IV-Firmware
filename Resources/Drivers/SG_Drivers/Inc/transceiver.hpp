/*
    transceiver.hpp
    Created May 2, 2024
        Yash Bhat

*/

#ifndef transceiver_HPP_
#define transceiver_HPP_

#include "main.h"
#include "sg_can.hpp"
class transceiver{
public:
    transceiver(UART_HandleTypeDef *huart);
    HAL_StatusTypeDef SendFrame(CANFrame& frame);
private:
    HAL_StatusTypeDef SendByte(uint8_t data);
    uint8_t EscapeData(uint8_t data);
    UART_HandleTypeDef *huart_;
    static constexpr uint8_t START_CHAR = 0xFF;
    static constexpr uint8_t ESC_CHAR = 0x2F;
    static constexpr uint8_t END_CHAR = 0x3F;
};

#endif /* transceiver_HPP_*/