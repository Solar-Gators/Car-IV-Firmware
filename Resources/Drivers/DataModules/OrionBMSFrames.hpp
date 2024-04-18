/*
 * OrionBMSFrames.hpp
 *
 * Created on: March 3, 2024
 *     Author: Jackson Werner
 */

#pragma once

#include "DataModule.hpp"
// Sunrider-Firmware uses little endianness
DATAMODULE( 

    BMSFrame0,
    0x6B0,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,
    
    // all voltages in V * 1e-4
    inline static uint16_t GetPackVoltage() {
        return ((Data()[6] << 8) | Data()[7]);
    }

    inline static uint16_t GetAvgCellVoltage() {
        return ((Data()[4] << 8) | Data()[5]);
    }

    inline static uint16_t GetHighCellVoltage() {
        return ((Data()[2] << 8) | Data()[3]);
    }

    inline static uint16_t GetLowCellVoltage() {
        return ((Data()[0] << 8) | Data()[1]);
    }

    // all voltages in V * 1e-4
    inline static void SetPackVoltage(uint16_t voltage) {
        Data()[6] = (voltage >> 8) & 0xFF;
        Data()[7] = voltage & 0xFF;
    }

    inline static void SetAvgCellVoltage(uint16_t voltage) {
        Data()[4] = (voltage >> 8) & 0xFF;
        Data()[5] = voltage & 0xFF;
    }

    inline static void SetHighCellVoltage(uint16_t voltage) {
        Data()[2] = (voltage >> 8) & 0xFF;
        Data()[3] = voltage & 0xFF;
    }

    inline static void SetLowCellVoltage(uint16_t voltage) {
        Data()[0] = (voltage >> 8) & 0xFF;
        Data()[1] = voltage & 0xFF;
    }
)

DATAMODULE(

    BMSFrame1,
    0x6B1,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    inline static uint8_t GetConstantVal() { // don't think this is necessary
        return (Data()[6] << 8) | Data()[7]; 
    }

    inline static uint8_t GetInternalTemp() {
        return Data()[5];
    }
    inline static uint8_t GetAverageTemp() {
        return Data()[4];
    }
    inline static uint8_t GetLowTempCellID() {
        return Data()[3];
    }
    inline static uint8_t GetLowTemp() {
        return Data()[2];
    }
    inline static uint8_t GetHighTempCellID() {
        return Data()[1];
    }
    inline static uint8_t GetHighTemp() {
        return Data()[0];
    }
)

DATAMODULE(

    BMSFrame2,
    0x6B2,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    inline static uint8_t GetConstantVal() { // don't think this is necessary
        return (Data()[6] << 8) | Data()[7]; 
    }

    inline static uint16_t GetPackCurrent() { // pack current in amps*10 (divide by 10 to get amperage)
        return ((Data()[4] << 8) | Data()[5]);
    }

    inline static uint16_t GetPackCCL() { // pack charge current limit in amps
        return ((Data()[2] << 8) | Data()[3]);
    }

    inline static uint16_t GetPackDCL() { // pack discharge current limit in amps
        return ((Data()[0] << 8) | Data()[1]);
    }
)

DATAMODULE(

    BMSFrame3,
    0x6B3,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    inline static uint16_t GetPackRes() { // pack internal resistance in mOhms
        return ((Data()[4] << 8) | Data()[5]);
    }

    inline static uint16_t HighCellRes() { // Highest internal resistance in pack (Ohm * 0.01 according to Sunrider-Firmware)
        return ((Data()[2] << 8) | Data()[3]);
    }

    inline static uint16_t LowCellRes() { // Lowest internal resistance in pack (Ohm * 0.01 according to Sunrider-Firmware)
        return ((Data()[0] << 8) | Data()[1]);
    }
)

DATAMODULE(

    BMSFrame4,
    0x6B4,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    inline static uint8_t GetFaultFlags0() {
        return Data()[0];
    }
    /*
    internal_cell_communication_fault_  = Data()[0] & (1 << 0);
    cell_balancing_stuck_off_fault_     = Data()[0] & (1 << 1);
    weak_cell_fault_                    = Data()[0] & (1 << 2);
    low_cell_voltage_fault_             = Data()[0] & (1 << 3);
    cell_open_wiring_fault_             = Data()[0] & (1 << 4);
    current_sensor_fault_               = Data()[0] & (1 << 5);
    cell_voltage_over_5v_fault_         = Data()[0] & (1 << 6);
    cell_bank_fault_                    = Data()[0] & (1 << 7);*/

    inline static uint8_t GetFaultFlags1() {
        return Data()[1];
    }
    /*
    weak_pack_fault_                    = Data()[1] & (1 << 0);
    fan_monitor_fault_                  = Data()[1] & (1 << 1);
    thermistor_fault_                   = Data()[1] & (1 << 2);
    can_communication_fault_            = Data()[1] & (1 << 3);
    redundant_power_supply_fault_       = Data()[1] & (1 << 4);
    high_voltage_isolation_fault_       = Data()[1] & (1 << 5);
    invalid_input_supply_voltage_fault_ = Data()[1] & (1 << 6);
    chargeenable_relay_fault_           = Data()[1] & (1 << 7);*/

    inline static uint8_t GetFaultFlags2(){
        return Data()[2];
    }
    /*
    dischargeenable_relay_fault_        = Data()[2] & (1 << 0);
    charger_safety_relay_fault_         = Data()[2] & (1 << 1);
    internal_hardware_fault_            = Data()[2] & (1 << 2);
    internal_heatsink_thermistor_fault_ = Data()[2] & (1 << 3);
    internal_logic_fault_               = Data()[2] & (1 << 4);
    highest_cell_voltage_too_high_fault_= Data()[2] & (1 << 5);
    lowest_cell_voltage_too_low_fault_  = Data()[2] & (1 << 6);
    pack_too_hot_fault_                 = Data()[2] & (1 << 7);*/

    inline static uint8_t GetPackSOC() {
        return Data()[3];
    }
)

DATAMODULE(

    BMSFrame5,
    0x6B5,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    inline static uint8_t GetMinPackVoltage() {  // in 0.1V (multiply by 0.1)
        return (Data()[6] << 8) | Data()[7]; 
    }

    inline static uint16_t GetMaxPackVoltage() { // in 0.1V (multiply by 0.1)
        return ((Data()[4] << 8) | Data()[5]);
    }

    inline static uint16_t GetMaxPackCCL() { 
        return ((Data()[2] << 8) | Data()[3]);
    }

    inline static uint16_t GetMaxPackDCL() { 
        return ((Data()[0] << 8) | Data()[1]);
    }
)