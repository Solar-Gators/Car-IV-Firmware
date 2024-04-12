/*
 * CustomBMSFrames.hpp
 *
 *  Created on: April 4, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"
#include <string.h>

DATAMODULE(
    
    BMSVoltageFrame,
    0x6C0,
    CAN_ID_STD,
    CAN_RTR_DATA,
    6,

    // Voltage in V * 1e-1
    inline static uint16_t GetPackVoltage() {
        return ((Data()[1] << 8) | Data()[0]);
    }

    // Voltage in mV
    inline static uint16_t GetAvgCellVoltage() {
        return ((Data()[2] << 8) | Data()[1]);
    }

    // Voltage in mV
    inline static uint16_t GetHighCellVoltage() {
        return ((Data()[4] << 8) | Data()[3]);
    }

    // Voltage in mV
    inline static uint16_t GetLowCellVoltage() {
        return ((Data()[6] << 8) | Data()[5]);
    }

    // Voltage in V * 1e-1
    inline static void SetPackVoltage(uint16_t voltage) {
        memcpy(Data(), &voltage, 2);
    }

    // Voltage in mV
    inline static void SetAvgCellVoltage(uint16_t voltage) {
        memcpy(Data() + 2, &voltage, 2);
    }

    // Voltage in mV
    inline static void SetHighCellVoltage(uint16_t voltage) {
        memcpy(Data() + 4, &voltage, 2);
    }

    // Voltage in mV
    inline static void SetLowCellVoltage(uint16_t voltage) {
        memcpy(Data() + 6, &voltage, 2);
    }
)        

DATAMODULE (

    BMSCurrentFrame,
    0x6C1,
    CAN_ID_STD,
    CAN_RTR_DATA,
    4,

    // Current in A * 1e-2
    inline static int32_t GetPackCurrent() {
        return ((Data()[3] << 24) | (Data()[2] << 16) | (Data()[1] << 8) | Data()[0]);
    }

    // Current in A * 1e-2
    inline static void SetPackCurrent(int32_t current) {
        memcpy(Data(), &current, 4);
    }
)

DATAMODULE(

    BMSTemperatureFrame,
    0x6C2,
    CAN_ID_STD,
    CAN_RTR_DATA,
    6,

    // Temperature in C
    inline static uint8_t GetInternalTemp() {
        return Data()[5];
    }

    // Temperature in C
    inline static uint8_t GetAvgTemp() {
        return Data()[4];
    }

    // ID of the cell with the lowest temperature
    inline static uint8_t GetLowTempID() {
        return Data()[3];
    }

    // Temperature in C
    inline static uint8_t GetLowTemp() {
        return Data()[2];
    }

    // ID of the cell with the highest temperature
    inline static uint8_t GetHighTempID() {
        return Data()[1];
    }

    // Temperature in C
    inline static uint8_t GetHighTemp() {
        return Data()[0];
    }
)

DATAMODULE (

    BMSFlagsFrame,
    0x6C3,
    CAN_ID_STD,
    CAN_RTR_DATA,
    3,

    // SoC in %
    inline static uint8_t GetPackSoC() {
        return Data()[2];
    }

    /*
     * Bit 0: Low Cell Voltage
     * Bit 1: High Cell Voltage
     * Bit 2: High Discharge Current
     * Bit 3: High Charge Current
     * Bit 4: High Temp
     * Bit 5: Thermistor Disconnected
     * Bit 6: Current Sensor Disconnected
     * Bit 7: Reserved
    */
    inline static uint8_t GetFaultFlags() {
        return Data()[1];
    }

    inline static bool GetLowCellVoltageFault() {
        return Data()[1] & 0b1;
    }

    inline static bool GetHighCellVoltageFault() {
        return Data()[1] & 0b10;
    }

    inline static bool GetHighDischargeCurrentFault() {
        return Data()[1] & 0b100;
    }

    inline static bool GetHighChargeCurrentFault() {
        return Data()[1] & 0b1000;
    }

    inline static bool GetHighTempFault() {
        return Data()[1] & 0b10000;
    }

    inline static bool GetThermistorDisconnectedFault() {
        return Data()[1] & 0b100000;
    }

    inline static bool GetCurrentSensorDisconnectedFault() {
        return Data()[1] & 0b1000000;
    }

    /*
     * Bit 0: Contactor 0 Status
     * Bit 1: Contactor 1 Status
     * Bit 2: Contactor 2 Status
     * Bit 3: Contactor 3 Status
     * Bit 4: Contactor Source (0: Main, 1: Supp)
     * Bit 5: Balancing Active
     * Bit 6: Reserved
     * Bit 7: Reserved
    */
    inline static uint8_t GetStatusFlags() {
        return Data()[0];
    }

    inline static bool GetContactorStatus(uint8_t contactor) {
        return Data()[0] & (0b1 << contactor);
    }

    inline static bool GetContactorSource() {
        return Data()[0] & 0b10000;
    }

    inline static bool GetBalancingActive() {
        return Data()[0] & 0b100000;
    }

    /*
     * Bit 0: Low Cell Voltage
     * Bit 1: High Cell Voltage
     * Bit 2: High Discharge Current
     * Bit 3: High Charge Current
     * Bit 4: High Temp
     * Bit 5: Thermistor Disconnected
     * Bit 6: Current Sensor Disconnected
     * Bit 7: Reserved
    */
    inline static void SetFaultFlags(uint8_t flags) {
        Data()[1] = flags;
    }

    inline static void SetLowCellVoltageFault(bool status) {
        uint8_t flag = 0b1;
        if (status) {
            Data()[1] |= flag;
        } else {
            Data()[1] &= ~flag;
        }
    }

    inline static void SetHighCellVoltageFault(bool status) {
        uint8_t flag = 0b10;
        if (status) {
            Data()[1] |= 0b10;
        } else {
            Data()[1] &= ~flag;
        }
    }

    inline static void SetHighDischargeCurrentFault(bool status) {
        uint8_t flag = 0b100;
        if (status) {
            Data()[1] |= flag;
        } else {
            Data()[1] &= ~flag;
        }
    }

    inline static void SetHighChargeCurrentFault(bool status) {
        uint8_t flag = 0b1000;
        if (status) {
            Data()[1] |= flag;
        } else {
            Data()[1] &= ~flag;
        }
    }

    inline static void SetHighTempFault(bool status) {
        uint8_t flag = 0b10000;
        if (status) {
            Data()[1] |= flag;
        } else {
            Data()[1] &= ~flag;
        }
    }

    inline static void SetThermistorDisconnectedFault(bool status) {
        uint8_t flag = 0b100000;
        if (status) {
            Data()[1] |= flag;
        } else {
            Data()[1] &= ~flag;
        }
    }

    inline static void SetCurrentSensorDisconnectedFault(bool status) {
        uint8_t flag = 0b1000000;
        if (status) {
            Data()[1] |= flag;
        } else {
            Data()[1] &= ~flag;
        }
    }

    /*
     * Bit 0: Contactor 0 Status
     * Bit 1: Contactor 1 Status
     * Bit 2: Contactor 2 Status
     * Bit 3: Contactor 3 Status
     * Bit 4: Contactor Source (0: Main, 1: Supp)
     * Bit 5: Balancing Active
     * Bit 6: Reserved
     * Bit 7: Reserved
    */
    inline static void SetStatusFlags(uint8_t flags) {
        Data()[0] = flags;
    }

    inline static void SetContactorStatus(uint8_t contactor, bool status) {
        uint8_t flag = 0b1 << contactor;
        if (status) {
            Data()[0] |= flag;
        } else {
            Data()[0] &= ~flag;
        }
    }

    inline static void SetContactorSource(bool source) {
        uint8_t flag = 0b10000;
        if (source) {
            Data()[0] |= flag;
        } else {
            Data()[0] &= ~flag;
        }
    }

    inline static void SetBalancingActive(bool status) {
        uint8_t flag = 0b100000;
        if (status) {
            Data()[0] |= flag;
        } else {
            Data()[0] &= ~flag;
        }
    }
)

DATAMODULE(

    BMSSecondaryFrame0,
    0x6C4,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Voltage in V * 1e-1
    inline static uint16_t GetSubpackVoltage() {
        return ((Data()[6] << 8) | Data()[7]);
    }

    // Voltage in mV
    inline static uint16_t GetAverageVoltage() {
        return ((Data()[4] << 8) | Data()[5]);
    }

    // Voltage in mV
    inline static uint16_t GetHighVoltage() {
        return ((Data()[2] << 8) | Data()[3]);
    }

    // Voltage in mV
    inline static uint16_t GetLowVoltage() {
        return ((Data()[0] << 8) | Data()[1]);
    }

    // Voltage in mV
    inline static void SetSubpackVoltage(uint16_t voltage) {
        memcpy(Data() + 6, &voltage, 2);
    }

    // Voltage in mV
    inline static void SetAverageVoltage(uint16_t voltage) {
        memcpy(Data() + 4, &voltage, 2);
    }

    // Voltage in mV
    inline static void SetHighVoltage(uint16_t voltage) {
        memcpy(Data() + 2, &voltage, 2);
    }

    // Voltage in mV
    inline static void SetLowVoltage(uint16_t voltage) {
        memcpy(Data(), &voltage, 2);
    }
)

DATAMODULE(

    BMSSecondaryFrame1,
    0x6C5,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Temperature in C
    inline static uint8_t GetInternalTemp() {
        return Data()[5];
    }

    // Temperature in C
    inline static uint8_t GetAverageTemp() {
        return Data()[4];
    }

    // ID of the cell with the lowest temperature
    inline static uint8_t GetLowTempCellID() {
        return Data()[3];
    }

    // Temperature in C
    inline static uint8_t GetLowTemp() {
        return Data()[2];
    }

    // ID of the cell with the highest temperature
    inline static uint8_t GetHighTempCellID() {
        return Data()[1];
    }

    // Temperature in C
    inline static uint8_t GetHighTemp() {
        return Data()[0];
    }

    // Temperature in C
    inline static void SetInternalTemp(uint8_t temp) {
        Data()[5] = temp;
    }

    // Temperature in C
    inline static void SetAverageTemp(uint8_t temp) {
        Data()[4] = temp;
    }

    // ID of the cell with the lowest temperature
    inline static void SetLowTempCellID(uint8_t cell) {
        Data()[3] = cell;
    }

    // Temperature in C
    inline static void SetLowTemp(uint8_t temp) {
        Data()[2] = temp;
    }

    // ID of the cell with the highest temperature
    inline static void SetHighTempCellID(uint8_t cell) {
        Data()[1] = cell;
    }

    // Temperature in C
    inline static void SetHighTemp(uint8_t temp) {
        Data()[0] = temp;
    }
)

DATAMODULE(

    BMSSecondaryFrame2,
    0x6C6,
    CAN_ID_STD,
    CAN_RTR_DATA,
    2,

    inline static uint8_t GetNumCells() {
        return Data()[1];
    }

    /*
     * Bit 0: Thermistor Disconnected
     * Bit 1: Cell Balancing Status
     * Bit 2: Reserved
     * Bit 3: Reserved
     * Bit 4: Reserved
     * Bit 5: Reserved
     * Bit 6: Reserved
     * Bit 7: Reserved
     */
    inline static uint8_t GetStatusFlags() {
        return Data()[0];
    }

    inline static void SetNumCells(uint8_t num) {
        Data()[1] = num;
    }

    /*
     * Bit 0: Thermistor Disconnected
     * Bit 1: Cell Balancing Status
     * Bit 2: Reserved
     * Bit 3: Reserved
     * Bit 4: Reserved
     * Bit 5: Reserved
     * Bit 6: Reserved
     * Bit 7: Reserved
     */
    inline static void SetStatusFlags(uint8_t flags) {
        Data()[0] = flags;
    }
)