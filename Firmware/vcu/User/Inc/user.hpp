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
#include "transceiver.hpp"

#include "VCUFrame.hpp"
#include "DriverControls.hpp"
#include "IoTestFrame.hpp"
//#include "OrionBMSFrames.hpp"
#include "CustomBMSFrames.hpp"
#include "MitsubaFrames.hpp"
#include "MPPTFrames.hpp"

#endif /* USER_HPP_ */

/* Externs */
extern FIL fil;         // Log file
extern Button kill_sw;  // Kill switch
extern bool kill_state; // Kill switch state
extern bool bms_trip;   // BMS trip state
extern bool sd_present; // SD card is inserted

typedef struct {
    uint16_t MAX_REGEN_VAL;
    uint16_t MAX_THROTTLE_VAL;
} VCUConfig_t;

const VCUConfig_t vcu_config = {
    .MAX_REGEN_VAL = 16,
    .MAX_THROTTLE_VAL = 16,
};

bool SD_Init();
void SetMotorState(bool state);
void SetMotorMode(bool mode);
void SetMotorDirection(bool direction);
void SetMPPTState(bool state);
void SetThrottle(uint16_t val);
void SetRegen(uint16_t val);