#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "threads.h"
// #include "usbd_cdc_if.h"

#include "sg_can.hpp"
#include "ADS7138.hpp"

/* Datamodules */
#include "DriverControls.hpp"
#include "MotorControlFrame.hpp"

extern "C" I2C_HandleTypeDef hi2c2;
extern "C" ADC_HandleTypeDef hadc2;

uint16_t readThrottleValue();

#endif /* USER_HPP_ */
