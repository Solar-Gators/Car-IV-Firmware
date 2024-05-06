#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "threads.h"
// #include "usbd_cdc_if.h"

#include "sg_can.hpp"

/* Datamodules */
#include "DriverControls.hpp"
#include "MotorControlFrame.hpp"

extern "C" I2C_HandleTypeDef hi2c2;

#endif /* USER_HPP_ */
