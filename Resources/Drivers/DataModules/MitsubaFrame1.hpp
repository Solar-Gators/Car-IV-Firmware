/*
 * MitsubaFrame1.hpp
 *
 *  Created on: February 11, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include <stdint.h>
#include <sg_can.hpp>

class MitsubaFrame1 : public CANFrame {
public:
    MitsubaFrame1() : CANFrame(0x08850225, CAN_ID_EXT, CAN_RTR_REMOTE, 8) {}

    inline uint16_t GetBatteryVoltage() {
        return static_cast<uint16_t>(((this->data[1] & 0x3) | (this->data[0])) / 2);
    }

    inline int32_t GetBatteryCurrent() {
        return 10;
    }
};

inline MitsubaFrame1 mitsuba_frame_1 = MitsubaFrame1();