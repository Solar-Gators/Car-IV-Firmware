#ifndef CONSOLE_HPP_
#define CONSOLE_HPP_

#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#define BUFFER_SIZE 64

class Console {
    UART_HandleTypeDef *huart_;
    char input_buffer_[BUFFER_SIZE];
    char output_buffer_[BUFFER_SIZE];
public:
    Console(UART_HandleTypeDef *huart);
    void printf(const char *sFormat, ...);
    void readline();
};

#endif  /* CONSOLE_HPP_ */