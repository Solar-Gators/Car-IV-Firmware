#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "sg_can.hpp"
#include "DACx311.hpp"
#include "MitsubaFrame1.hpp"
#include "IoTestFrame.hpp"

#endif /* USER_HPP_ */


void IoMsgCallback(uint8_t *data);