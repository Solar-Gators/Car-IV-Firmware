/*
 * IoTestFrame.hpp
 *
 *  Created on: March 25, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

DATAMODULE(
    
    VCUFrame0,
    0x010,
    CAN_ID_STD,
    CAN_RTR_DATA,
    1,

    inline static bool GetKillStatus() {
        return static_cast<bool>(Data()[0] & 0x1);
    }

    inline static void SetKillStatus(bool status) {
        Data()[0] = status & 0x1;
    }
)