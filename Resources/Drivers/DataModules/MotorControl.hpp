/*
 * MotorControl.hpp
 *
 *  Created on: February 12, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include <stdint.h>
#include <sg_can.hpp>

#define MOTOR_MAIN_EN_FLAG (0x01 << 0)
#define MOTOR_ECO_FLAG (0x01 << 1)
#define MOTOR_REVERSE_FLAG (0x01 << 2)
#define BRAKE_EN_FLAG (0x01 << 3)

class MotorControl : public CANFrame {
public:
    MotorControl() : CANFrame(0x234, CAN_ID_STD, CAN_RTR_DATA, 8) {}

    inline uint16_t GetThrottleVal() {
        return static_cast<uint16_t>((this->data[1] << 8) | this->data[0]);
    }

    inline uint16_t GetRegenVal() {
        return static_cast<uint16_t>((this->data[3] << 8) | this->data[2]);
    }

    inline uint8_t StatusFlags() {
        return this->data[4];
    }

    inline void SetThrottleVal(uint16_t val) {
        this->data[1] = (val >> 8) & 0xFF;
        this->data[0] = val & 0xFF;
    }

    inline void SetRegenVal(uint16_t val) {
        this->data[3] = (val >> 8) & 0xFF;
        this->data[2] = val & 0xFF;
    }

    inline void SetStatusFlags(uint8_t flags) {
        this->data[4] = flags;
    }
};