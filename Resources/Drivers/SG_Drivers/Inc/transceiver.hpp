/*
    transceiver.hpp
    Created May 2, 2024
        Yash Bhat

*/

#ifndef transceiver_HPP_
#define transceiver_HPP_

#define PACKETLEN 13

#include "sg_can.hpp"
class transceiver{
public:
    transceiver(UART_HandleTypeDef *huart);
    HAL_StatusTypeDef SendFrame(CANFrame& frame);
private:
    HAL_StatusTypeDef SendByte(uint8_t* data);
    UART_HandleTypeDef *huart_;
    uint8_t packet[PACKETLEN];
};

#endif /* transceiver_HPP_*/