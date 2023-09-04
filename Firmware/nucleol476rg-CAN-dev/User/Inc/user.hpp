#ifndef USER_HPP_
#define USER_HPP_

#include "main.h"
#include <stddef.h>

#include "sg_can.hpp"
#include "logger.hpp"

extern "C" void CPP_UserSetup(void);
extern "C" CAN_HandleTypeDef hcan1;


#endif /* USER_HPP_ */