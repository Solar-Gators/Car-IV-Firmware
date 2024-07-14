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
        
        packet[0] = ((canid & 0xFF000000) >> 24);
        packet[1] = ((canid & 0x00FF0000) >> 16);
        packet[2] = ((canid & 0x0000FF00) >> 8);
        packet[3] = (canid & 0x000000FF);

        packet[4] = Frame.len;

        for(uint32_t i = 0; i < 8; i++){
            packet[i+5] = (data_[i]);
        }
        
        Frame.Unlock();
        SendByte(packet);
    }
    HAL_Delay(2);
    return status;
}

HAL_StatusTypeDef transceiver::SendByte(uint8_t* data){
    return HAL_UART_Transmit(huart_, data, PACKETLEN, HAL_MAX_DELAY);
}