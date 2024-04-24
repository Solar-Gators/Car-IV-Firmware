/*
 * DriverControls.hpp
 *
 *  Created on: February 21, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

DATAMODULE(
    
        DriverControlsFrame0,
        0x011,
        CAN_ID_STD,
        CAN_RTR_DATA,
        3,
    
        inline static uint16_t GetThrottleVal() {
            return static_cast<uint16_t>((Data()[1] << 8) | Data()[0]);
        }
    
        inline static uint8_t GetFlags() {
            return static_cast<uint8_t>(Data()[2]);
        }

        inline static bool GetBrake() {
            return static_cast<bool>(Data()[2] & 0x01);
        }

        inline static bool GetShutdownStatus() {
            return static_cast<bool>(Data()[2] & 0x02);
        }

        inline static void SetThrottleVal(uint16_t val) {
            Data()[0] = static_cast<uint8_t>(val);
            Data()[1] = static_cast<uint8_t>(val >> 8);
        }
    
        inline static void SetFlags(uint8_t flags) {
            Data()[2] = flags;
        }

        inline static void SetBrake(bool val) {
            if (val) {
                Data()[2] |= 0x01;
            } else {
                Data()[2] &= ~0x01;
            }
        }

        inline static void SetShutdownStatus(bool val) {
            if (val) {
                Data()[2] |= 0x02;
            } else {
                Data()[2] &= ~0x02;
            }
        }
)

enum class MotorMode : uint8_t {
    ECO = 0x0,
    POWER = 0x2,
    MASK = 0x2
};

enum class MotorDirection : uint8_t {
    FORWARD = 0x0,
    REVERSE = 0x4,
    MASK = 0x4
};

DATAMODULE(

    DriverControlsFrame1,
    0x012,
    CAN_ID_STD,
    CAN_RTR_DATA,
    1,

    inline static uint8_t GetFlags0() {
        return static_cast<uint8_t>(Data()[0]);
    }

    inline static bool GetHeadlight() {
        return static_cast<bool>(Data()[0] & 0x01);
    }

    inline static bool GetLeftTurn() {
        return static_cast<bool>(Data()[0] & 0x02);
    }

    inline static bool GetRightTurn() {
        return static_cast<bool>(Data()[0] & 0x04);
    }

    inline static bool GetHazards() {
        return static_cast<bool>(Data()[0] & 0x08);
    }

    inline static bool GetHorn() {
        return static_cast<bool>(Data()[0] & 0x10);
    }

    inline static bool GetPTT() {
        return static_cast<bool>(Data()[0] & 0x20);
    }

    inline static uint8_t GetFlags1() {
        Data()[1] = flags;
    }

    inline static bool GetMotorEnable() {
        return static_cast<bool>(Data()[1] & 0x01);
    }

    inline static bool GetDriveMode() {
        return static_cast<bool>(Data()[1] & MotorMode::MASK);
    }

    inline static bool GetDriveDirection() {
        return static_cast<bool>(Data()[1] & MotorDirection::MASK);
    }

    inline static bool GetRegenEnable() {
        return static_cast<bool>(Data()[1] & 0x08);
    }

    inline static bool GetPVEnable() {
        return static_cast<bool>(Data()[1] & 0x10);
    }

    inline static bool GetBMSEnable() {
        return static_cast<bool>(Data()[1] & 0x20);
    }

    inline static void SetFlags0(uint8_t flags) {
        Data()[0] = flags;
    }

    inline static void SetHeadlight(bool val) {
        if (val) {
            Data()[0] |= 0x01;
        } else {
            Data()[0] &= ~0x01;
        }
    }

    inline static void SetLeftTurn(bool val) {
        if (val) {
            Data()[0] |= 0x02;
        } else {
            Data()[0] &= ~0x02;
        }
    }

    inline static void SetRightTurn(bool val) {
        if (val) {
            Data()[0] |= 0x04;
        } else {
            Data()[0] &= ~0x04;
        }
    }

    inline static void SetHazards(bool val) {
        if (val) {
            Data()[0] |= 0x08;
        } else {
            Data()[0] &= ~0x08;
        }
    }

    inline static void SetHorn(bool val) {
        if (val) {
            Data()[0] |= 0x10;
        } else {
            Data()[0] &= ~0x10;
        }
    }

    inline static void SetPTT(bool val) {
        if (val) {
            Data()[0] |= 0x20;
        } else {
            Data()[0] &= ~0x20;
        }
    }

    inline static void SetFlags1(uint8_t flags) {
        Data()[1] = flags;
    }

    inline static void SetMotorEnable(bool val) {
        if (val) {
            Data()[1] |= 0x01;
        } else {
            Data()[1] &= ~0x01;
        }
    }

    inline static void SetDriveMode(MotorMode mode) {
        Data()[1] &= ~MotorMode::MASK;
        Data()[1] |= static_cast<uint8_t>(mode);
    }

    inline static void SetDriveDirection(MotorDirection direction) {
        Data()[1] &= ~MotorDirection::MASK;
        Data()[1] |= static_cast<uint8_t>(direction);
    }

    inline static void SetRegenEnable(bool val) {
        if (val) {
            Data()[1] |= 0x08;
        } else {
            Data()[1] &= ~0x08;
        }
    }

    inline static void SetPVEnable(bool val) {
        if (val) {
            Data()[1] |= 0x10;
        } else {
            Data()[1] &= ~0x10;
        }
    }

    inline static void SetBMSEnable(bool val) {
        if (val) {
            Data()[1] |= 0x20;
        } else {
            Data()[1] &= ~0x20;
        }
    }
)

DATAMODULE(

    DriverControlsFrame2,
    0x013,
    CAN_ID_STD,
    CAN_RTR_DATA,
    2,

    inline static uint16_t GetCruiseVal() {
        return static_cast<uint16_t>((Data()[1] << 8) | Data()[0]);
    }

    inline static void SetCruiseVal(uint16_t val) {
        Data()[0] = static_cast<uint8_t>(val);
        Data()[1] = static_cast<uint8_t>(val >> 8);
    }
)