#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"

#include "sg_can.hpp"
#include "logger.hpp"
#include "button.hpp"

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;

extern CANDevice can1;

void ButtonCallback(void);
void LongCallback(void);
void DoubleCallback(void);

#endif /* USER_HPP_ */
