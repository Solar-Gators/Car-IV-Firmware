/*
 * MitsubaFrames.hpp
 *
 *  Created on: February 28, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

DATAMODULE(

    /* CAN definitions */
    MitsubaRequestFrame,
    0x08F89540,
    CAN_ID_EXT,
    CAN_RTR_DATA,
    8,

    inline static void SetRequestAll() {
        Data()[0] |= 0x7;
    }

    inline static void SetRequest0() {
        Data()[0] |= (0x1 << 0);
    }

    inline static void SetRequest1() {
        Data()[0] |= (0x1 << 1);
    }

    inline static void SetRequest2() {
        Data()[0] |= (0x1 << 2);
    }

    inline static void ResetRequest0() {
        Data()[0] &= ~(0x1 << 0);
    }

    inline static void ResetRequest1() {
        Data()[0] &= ~(0x1 << 1);
    }

    inline static void ResetRequest2() {
        Data()[0] &= ~(0x1 << 2);
    }
)

DATAMODULE(

    MitsubaFrame0,
    0x08850225,
    CAN_ID_EXT,
    CAN_RTR_DATA,
    8,

    // Existing getters
    inline static uint16_t GetVoltage() {
        return ((Data()[1] & 0x3) << 8) | Data()[0];
    }

    inline static uint16_t GetBatteryCurrent() {
        return ((Data()[2] << 1) | (Data()[3] >> 7)) & 0x1FF;
    }

    // Battery Current Direction
    inline static bool GetBatteryCurrentDirection() {
        return (Data()[3] >> 6) & 0x1;
    }

    // Motor Current (A)
    inline static uint16_t GetMotorCurrent() {
        return ((Data()[3] & 0x3F) << 4) | (Data()[4] >> 4);
    }

    // FET Temperature (increments of 5C)
    inline static uint8_t GetFETTemp() {
        return (Data()[4] & 0xF) << 1 | (Data()[5] >> 7);
    }

    // Motor RPM (1RPM)
    inline static uint16_t GetMotorRPM() {
        return ((Data()[5] & 0x7F) << 5) | (Data()[6] >> 3);
    }

    // PWM Duty Cycle (0.5%)
    inline static uint16_t GetPWMDutyCycle() {
        return ((Data()[6] & 0x7) << 7) | (Data()[7] >> 1);
    }

    // Advanced Lead Angle
    inline static uint8_t GetAdvancedLeadAngle() {
        return Data()[7] & 0x7F;
    }
)

DATAMODULE(
    MitsubaFrame1,
    0x08950225,
    CAN_ID_EXT,
    CAN_RTR_DATA,
    5,

    // Power Mode
    inline static bool GetPowerMode() {
        return (Data()[0] >> 7) & 0x1;
    }

    // MC Mode
    inline static bool GetMCMode() {
        return (Data()[0] >> 6) & 0x1;
    }

    // Accelerator Position
    inline static uint16_t GetAcceleratorPosition() {
        return ((Data()[0] & 0x3F) << 4) | (Data()[1] >> 4);
    }

    // Regen VR Position
    inline static uint16_t GetRegenVRPosition() {
        return ((Data()[1] & 0xF) << 6) | (Data()[2] >> 2);
    }

    // Digit SW Position
    inline static uint8_t GetDigitSWPosition() {
        return ((Data()[2] & 0x3) << 2) | (Data()[3] >> 6);
    }

    // Output Target Value
    inline static uint16_t GetOutputTargetValue() {
        return ((Data()[3] & 0x3F) << 4) | (Data()[4] >> 4);
    }

    // Drive Action Status
    inline static uint8_t GetDriveActionStatus() {
        return (Data()[4] >> 2) & 0x3;
    }

    // Regen Status
    inline static bool GetRegenStatus() {
        return (Data()[4] >> 1) & 0x1;
    }
)

DATAMODULE(
    MitsubaFrame2,
    0x08A50225,
    CAN_ID_EXT,
    CAN_RTR_DATA,
    5,

    inline static  uint32_t GetAllFlags() {
        return (Data()[3] << 24) | (Data()[2] << 16) | (Data()[1] << 8) | Data()[0];
    }

    inline static bool GetAnalogSensorError() {
        return Data()[0] & (0x1 << 7);
    }

    inline static bool GetMotorCurrentSensorUError() {
        return Data()[0] & (0x1 << 6);
    }

    inline static bool GetMotorCurrentSensorWError() {
        return Data()[0] & (0x1 << 5);
    }

    inline static bool GetFETThermistorError() {
        return Data()[0] & (0x1 << 4);
    }

    inline static bool GetBatteryVoltageSensorError() {
        return Data()[0] & (0x1 << 3);
    }

    inline static bool GetBatteryCurrentSensorError() {
        return Data()[0] & (0x1 << 2);
    }

    inline static bool GetBatteryCurrentSensorAdjustError() {
        return Data()[0] & (0x1 << 1);
    }

    inline static bool GetMotorCurrentSensorAdjustError() {
        return Data()[1] & (0x1 << 7);
    }

    inline static bool GetAcceleratorPositionError() {
        return Data()[1] & (0x1 << 6);
    }

    inline static bool GetControllerVoltageSensorError() {
        return Data()[1] & (0x1 << 4);
    }

    inline static bool GetPowerSystemError() {
        return Data()[2] & (0x1 << 7);
    }

    inline static bool GetOverCurrentError() {
        return Data()[2] & (0x1 << 6);
    }

    inline static bool GetOverVoltageError() {
        return Data()[2] & (0x1 << 4);
    }

    inline static bool GetOverCurrentLimit() {
        return Data()[2] & (0x1 << 2);
    }

    inline static bool GetMotorSystemError() {
        return Data()[3] & (0x1 << 7);
    }

    inline static bool GetMotorLock() {
        return Data()[3] & (0x1 << 6);
    }

    inline static bool GetHallSensorShort() {
        return Data()[3] & (0x1 << 5);
    }

    inline static bool GetHallSensorOpen() {
        return Data()[3] & (0x1 << 4);
    }

    inline static uint8_t GetOverHeatLevel() {
        return (Data()[4] >> 6) & 0x3;
    }
)