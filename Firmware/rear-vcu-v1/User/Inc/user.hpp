#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "sg_can.hpp"
#include "DACx311.hpp"
#include "MotorControlFrame.hpp"
#include "IoTestFrame.hpp"

#endif /* USER_HPP_ */

void SetMotorState(bool state);
void SetMotorMode(bool mode);
void SetMotorDirection(bool direction);
void SetThrottle(uint16_t val);
void SetRegen(uint16_t val);