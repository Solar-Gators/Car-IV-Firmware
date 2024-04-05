/*
 * CustomBMSFrames.hpp
 *
 *  Created on: April 4, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

DATAMODULE(
    
    BMSFrame0,
    0x6C0,
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

DATAMODULE (

    BMSFrame1,
    0x6C2,
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

    inline static void SetConstantVal(uint16_t value) {
        Data()[6] = value & 0xFF;
        Data()[7] = (value >> 8) & 0xFF;
    }

    inline static void SetInternalTemp(uint8_t temp) {
        Data()[5] = temp;
    }

    inline static void SetAverageTemp(uint8_t temp) {
        Data()[4] = temp;
    }

    inline static void SetLowTempCellID(uint8_t cell) {
        Data()[3] = cell;
    }

    inline static void SetLowTemp(uint8_t temp) {
        Data()[2] = temp;
    }

    inline static void SetHighTempCellID(uint8_t cell) {
        Data()[1] = cell;
    }

    inline static void SetHighTemp(uint8_t temp) {
        Data()[0] = temp;
    }
)

DATAMODULE(

    BMSFrame2,
    0x6C2,
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

    inline static void SetContantVal(uint16_t value) {
        Data()[6] = value & 0xFF;
        Data()[7] = (value >> 8) & 0xFF;
    }

    inline static void SetPackCurrent(uint16_t current) {
        Data()[4] = current & 0xFF;
        Data()[5] = (current >> 8) & 0xFF;
    }

    inline static void SetPackCCL(uint16_t current_limit) {
        Data()[2] = current_limit & 0xFF;
        Data()[3] = (current_limit >> 8) & 0xFF;
    }    

    inline static void SetPackDCL(uint16_t current_limit) {
        Data()[0] = current_limit & 0xFF;
        Data()[1] = (current_limit >> 8) & 0xFF;
    }
)