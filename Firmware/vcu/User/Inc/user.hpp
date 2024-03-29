#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "fatfs.h"
#include "logger.hpp"
#include "sg_can.hpp"
#include "button.hpp"
#include "DACx311.hpp"

#include "VCUFrame.hpp"
#include "DriverControls.hpp"
#include "IoTestFrame.hpp"
#include "MitsubaFrames.hpp"

#endif /* USER_HPP_ */

/* Externs */
extern FIL fil;     // Log file

void SetMotorState(bool state);
void SetMotorMode(bool mode);
void SetMotorDirection(bool direction);
void SetThrottle(uint16_t val);
void SetRegen(uint16_t val);