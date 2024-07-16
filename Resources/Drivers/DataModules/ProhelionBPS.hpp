/*
 * IoTestFrame.hpp
 *
 *  Created on: Jul 16, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

DATAMODULE(
    
    ProhelionCurrentSenseFrame,
    0x3FA,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    inline static int32_t GetCurrent() {
        return (Data()[4]) | (Data()[5] << 8) | (Data()[6] << 16) | (Data()[7] << 24);
    }

    inline static void SetCurrent(int32_t current) {
        Data()[4] = current & 0xFF;
        Data()[5] = (current >> 8) & 0xFF;
        Data()[6] = (current >> 16) & 0xFF;
        Data()[7] = (current >> 24) & 0xFF;
    }
)