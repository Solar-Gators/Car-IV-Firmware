#pragma once

//#include "main.h"
#include "sg_can.hpp"

#define MAX_PACKET_LEN 26

#define START_CHAR 0xFF
#define ESC_CHAR 0x2F
#define END_CHAR 0x3F

class RFD900x {
public:
    RFD900x(UART_HandleTypeDef *huart);
    HAL_StatusTypeDef SendFrame(CANFrame &frame);
private:
    HAL_StatusTypeDef SendByte(uint8_t* data, const uint8_t packet_size);
    UART_HandleTypeDef *huart;
    uint8_t packet[MAX_PACKET_LEN];
};
