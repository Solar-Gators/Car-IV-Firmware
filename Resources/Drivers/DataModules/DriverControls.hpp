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
        5,
    
        inline static uint16_t GetThrottleVal() {
            return static_cast<uint16_t>((Data()[1] << 8) | Data()[0]);
        }
    
        inline static uint16_t GetRegenVal() {
            return static_cast<uint16_t>((Data()[3] << 8) | Data()[2]);
        }
    
        inline static uint8_t GetFlags() {
            return static_cast<uint8_t>(Data()[4]);
        }

        inline static bool GetMotorEnable() {
            return static_cast<bool>(Data()[4] & 0x01);
        }

        inline static bool GetBrakeEnable() {
            return static_cast<bool>(Data()[4] & 0x02);
        }

        inline static bool GetCruiseEnable() {
            return static_cast<bool>(Data()[4] & 0x04);
        }

        inline static bool GetDriveMode() {
            return static_cast<bool>(Data()[4] & 0x08);
        }

        inline static bool GetDriveDirection() {
            return static_cast<bool>(Data()[4] & 0x10);
        }

        inline static bool GetSolarEnable() {
            return static_cast<bool>(Data()[4] & 0x20);
        }
    
        inline static void SetThrottleVal(uint16_t val) {
            Data()[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
            Data()[0] = static_cast<uint8_t>(val & 0xFF);
        }
    
        inline static void SetRegenVal(uint16_t val) {
            Data()[3] = static_cast<uint8_t>((val >> 8) & 0xFF);
            Data()[2] = static_cast<uint8_t>(val & 0xFF);
        }
    
        inline static void SetFlags(uint8_t flags) {
            Data()[4] = flags;
        }

        inline static void SetMotorEnable(bool val) {
            if (val) {
                Data()[4] |= 0x01;
            } else {
                Data()[4] &= ~0x01;
            }
        }

        inline static void SetBrakeEnable(bool val) {
            if (val) {
                Data()[4] |= 0x02;
            } else {
                Data()[4] &= ~0x02;
            }
        }

        inline static void SetCruiseEnable(bool val) {
            if (val) {
                Data()[4] |= 0x04;
            } else {
                Data()[4] &= ~0x04;
            }
        }

        inline static void SetDriveMode(bool val) {
            if (val) {
                Data()[4] |= 0x08;
            } else {
                Data()[4] &= ~0x08;
            }
        }

        inline static void SetDriveDirection(bool val) {
            if (val) {
                Data()[4] |= 0x10;
            } else {
                Data()[4] &= ~0x10;
            }
        }

        inline static void SetSolarEnable(bool val) {
            if (val) {
                Data()[4] |= 0x20;
            } else {
                Data()[4] &= ~0x20;
            }
        }
)

DATAMODULE(

    DriverControlsFrame1,
    0x012,
    CAN_ID_STD,
    CAN_RTR_DATA,
    1,

    inline static uint8_t GetFlags() {
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

    inline static void SetFlags(uint8_t flags) {
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
)