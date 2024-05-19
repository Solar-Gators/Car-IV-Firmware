/*
 * PowerBoard.hpp
 *
 *  Created on: May 19, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

enum class PowerSource_t : uint8_t {
    SUPPLEMENTAL = 0,
    MAIN = 1,
};

DATAMODULE(
    
        PowerBoardFrame,
        0x235,
        CAN_ID_STD,
        CAN_RTR_DATA,
        6,
    
        // Voltage in V / 100
        inline static uint16_t GetSuppBattVoltage() {
            return static_cast<uint16_t>((Data()[0] << 8) | Data()[1]);
        }

        // Power in W / 100
        inline static uint16_t GetSuppBattPower() {
            return static_cast<uint16_t>((Data()[2] << 8) | Data()[3]);
        }

        // Power in W / 100
        inline static uint16_t GetMainBattPower() {
            return static_cast<uint16_t>((Data()[4] << 8) | Data()[5]);
        }

        // Power MUX current selection
        inline static PowerSource_t GetPowerSource() {
            return static_cast<PowerSource_t>(Data()[6] & 0x01);
        }
)