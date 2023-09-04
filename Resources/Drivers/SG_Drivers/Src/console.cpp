#include "console.hpp"

Console::Console(UART_HandleTypeDef *huart) {
    huart_ = huart;
}

void Console::printf(const char *sFormat, ...) {
    va_list args;
    va_start(args, sFormat);

    // Write the formatted string to output_buffer_
    uint16_t len = vsnprintf(output_buffer_, BUFFER_SIZE, sFormat, args);

    va_end(args);

    // Transmit the formatted string over UART
    HAL_UART_Transmit(huart_, (uint8_t*)output_buffer_, len, HAL_MAX_DELAY);
}

void Console::readline() {
    uint32_t bufferIndex = 0;

    // Receive data over UART until carriage return '\r' is detected or buffer size overflow
    do {
        // Receive one character
        HAL_UART_Receive(huart_, (uint8_t*)(input_buffer_ + bufferIndex), 1, HAL_MAX_DELAY);

        switch (input_buffer_[bufferIndex]) {
            case '\b':  // backspace
                if (bufferIndex > 0) {
                    bufferIndex--;
                    HAL_UART_Transmit(huart_, (uint8_t*)"\b", 1, HAL_MAX_DELAY);
                }
                break;
            default:    // echo character
                HAL_UART_Transmit(huart_, (uint8_t*)(input_buffer_ + bufferIndex), 1, HAL_MAX_DELAY);
                bufferIndex++;
                break;
        }

        // Echo character
    } while (input_buffer_[bufferIndex-1] != '\r' && bufferIndex < BUFFER_SIZE);

    // Add null terminator
    input_buffer_[bufferIndex] = '\0';

    
}