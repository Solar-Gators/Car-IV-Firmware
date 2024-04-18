#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "etl/unordered_map.h"
#include "etl/string.h"

#include "vcom.h"
#include "sg_can.hpp"
#include "logger.hpp"
#include "button.hpp"

// Datamodules
#include "IoTestFrame.hpp"
#include "MitsubaFrames.hpp"
#include "DriverControls.hpp"


#define TOKEN_MAX_SIZE 12
extern const etl::unordered_map<etl::string<TOKEN_MAX_SIZE>, 
                                void(*)(), 
                                6> command_map;
#endif /* USER_HPP_ */
