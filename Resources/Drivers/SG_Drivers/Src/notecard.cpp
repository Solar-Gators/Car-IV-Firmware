#include "notecard.hpp"

Notecard::Notecard(UART_HandleTypeDef *huart) {
    this->huart = huart;
}

HAL_StatusTypeDef Notecard::SendCommand(const char *command) {
    HAL_StatusTypeDef status = HAL_OK;
    status = SendByte((uint8_t*)command, strlen(command));

    return status;
}

HAL_StatusTypeDef Notecard::LoadFrame(CANFrame& Frame) {
    uint32_t canid = Frame.can_id;
    uint8_t* data_ = Frame.data;
    HAL_StatusTypeDef status = HAL_OK;

    if(Frame.Lock() == osOK){
        uint8_t frame_len = Frame.len;

        // create frame data array
        uint8_t offset = 0;
        offset += sprintf(frame_data + offset, "[");
        
        for (uint8_t i = 0; i < frame_len; i++) {
            offset += sprintf(frame_data + offset, "%u", data_[i]);
            if (i < frame_len - 1) {
                offset += sprintf(frame_data + offset, ",");
            }
        }
        offset += sprintf(frame_data + offset, "]");
        frame_data[offset] = '\0';

        // create JSON packet
        uint8_t packet_len = sprintf((char*)json_packet, JSON_CAN_FORMAT, canid, frame_len, frame_data);
        
        Frame.Unlock();
        status = SendByte(json_packet, packet_len);
    }
    HAL_Delay(2);
    return status;
}

HAL_StatusTypeDef Notecard::SendByte(uint8_t *data, uint8_t len){
    return HAL_UART_Transmit(huart, data, len, HAL_MAX_DELAY);
}