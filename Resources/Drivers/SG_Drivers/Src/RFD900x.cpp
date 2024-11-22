#include "RFD900x.hpp"

RFD900x::RFD900x(UART_HandleTypeDef *huart) {
    this->huart = huart;
}

HAL_StatusTypeDef RFD900x::SendFrame(CANFrame &frame) {
    uint32_t canid = frame.can_id;
    uint8_t* data_ = frame.data;
    HAL_StatusTypeDef status = HAL_OK;

    if(frame.Lock() == osOK){
        uint8_t index = 0;

        packet[index++] = START_CHAR;
        packet[index++] = ((canid & 0xFF000000) >> 24);
        packet[index++] = ((canid & 0x00FF0000) >> 16);
        packet[index++] = ((canid & 0x0000FF00) >> 8);
        packet[index++] = ((canid & 0x000000FF) >> 0);
        packet[index++] = frame.len;

        for (int i = 0; i < 8; i++) {
            if (data_[i] == START_CHAR || data_[i] == END_CHAR || data_[i] == ESC_CHAR) {
                packet[index++] = ESC_CHAR;
            }
            packet[index++] = data_[i];
        }
        
        packet[index++] = END_CHAR;
        
        frame.Unlock();
        status = SendByte(packet, index);
    }
    HAL_Delay(2);
    return status;
}

HAL_StatusTypeDef RFD900x::SendByte(uint8_t* data, const uint8_t packet_size) {
    return HAL_UART_Transmit(huart, data, packet_size, HAL_MAX_DELAY);
}