/*
 * MotorControlFrame.hpp
 *
 *  Created on: February 12, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include <stdint.h>
#include <sg_can.hpp>
#include "DataModule.hpp"


DATAMODULE(MotorControlFrame, 0x234, CAN_ID_STD, CAN_RTR_DATA, 8, 
    inline static uint16_t GetThrottleVal() {
        return static_cast<uint16_t>((Data()[1] << 8) | Data()[0]);
    }

    inline static uint16_t GetRegenVal() {
        return static_cast<uint16_t>((Data()[3] << 8) | Data()[2]);
    }

    inline static uint8_t StatusFlags() {
        return Data()[4];
    }

    inline static void SetThrottleVal(uint16_t val) {
        Data()[1] = (val >> 8) & 0xFF;
        Data()[0] = val & 0xFF;
    }

    inline static void SetRegenVal(uint16_t val) {
        Data()[3] = (val >> 8) & 0xFF;
        Data()[2] = val & 0xFF;
    }

    inline static void SetStatusFlags(uint8_t flags) {
        Data()[4] = flags;
    }
)