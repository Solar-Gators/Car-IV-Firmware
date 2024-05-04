#include "transceiver.hpp"

transceiver::transceiver(UART_HandleTypeDef *huart){
    huart_ = huart;
}

HAL_StatusTypeDef transceiver::SendFrame(CANFrame& Frame){
    uint32_t canid = Frame.can_id;
    uint8_t* data_ = Frame.data;
    HAL_StatusTypeDef status = HAL_OK;
    if(Frame.Lock() == osOK){
        //send CAN ID
        status = SendByte(EscapeData((canid & 0xFF000000) >> 24));
        status = SendByte(EscapeData((canid & 0x00FF0000) >> 16));
        status = SendByte(EscapeData((canid & 0x0000FF00) >> 8));
        status = SendByte(EscapeData(canid & 0x000000FF));

        status = SendByte(EscapeData(Frame.len));

        for(uint32_t i = 0; i < Frame.len; i++){
            status = SendByte(EscapeData(data_[i]));
        }
        status = SendByte(END_CHAR);
        Frame.Unlock();
    }
    return status;
}

uint8_t transceiver::EscapeData(uint8_t data){
    if(data == START_CHAR|| data == END_CHAR || data == ESC_CHAR){
        HAL_UART_Transmit(huart_, &data, 1, HAL_MAX_DELAY);
    }
    return data;
}

HAL_StatusTypeDef transceiver::SendByte(uint8_t data){
    return HAL_UART_Transmit(huart_, &data, 1, HAL_MAX_DELAY);
}