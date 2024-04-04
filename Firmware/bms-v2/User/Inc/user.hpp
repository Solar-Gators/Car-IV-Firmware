#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "sg_can.hpp"
#include "BQ76952.hpp"
#include "ADS7138.hpp"

/* Datamodules */
#include "OrionBMSFrames.hpp"   // For now just replicate Orion frames

extern BQ76952 bms;
extern ADS7138 adcs[3];

void SetAmplifierState(bool state);

#endif /* USER_HPP_ */