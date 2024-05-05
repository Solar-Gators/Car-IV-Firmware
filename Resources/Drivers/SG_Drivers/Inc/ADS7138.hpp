/*
 *  ADS7138.hpp
 *
 *  Created on: February 21, 2024
 *      Author: Matthew Shen
 * 
 * https://www.ti.com/lit/ds/symlink/ads7138.pdf?ts=1708572728805&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS7138
 * 
 */

#pragma once

#include "main.h"
#include "logger.hpp"

/* ADS7138 opcodes */
enum class ADS7138_Opcode : uint8_t {
    READ =  0x10,
    WRITE = 0x08,
};

/* ADS7138 registers */
enum class ADS7138_Register : uint8_t {
    SYSTEM_STATUS =           0x0,
    GENERAL_CFG =             0x1,
    DATA_CFG =                0x2,
    OSR_CFG =                 0x3,
    OPMODE_CFG =              0x4,
    PIN_CFG =                 0x5,
    GPIO_CFG =                0x7,
    GPO_DRIVE_CFG =           0x9,
    GPO_VALUE =               0xB,
    GPI_VALUE =               0xD,
    SEQUENCE_CFG =            0x10,
    MANUAL_CH_SEL =           0x11,
    AUTO_SEQ_CH_SEL =         0x12,
    ALERT_CH_SEL =            0x14,
    ALERT_FUNC_SEL =          0x16,
    ALERT_PIN_CFG =           0x17,
    EVENT_FLAG =              0x18,
    EVENT_HIGH_FLAG =         0x1A,
    EVENT_LOW_FLAG =          0x1C,
    EVENT_RGN =               0x1E,
    HYSTERESIS_CH0 =          0x20,
    HIGH_TH_CH0 =             0x21,
    EVENT_COUNT_CH0 =         0x22,
    LOW_TH_CH0 =              0x23,
    HYSTERESIS_CH1 =          0x24,
    HIGH_TH_CH1 =             0x25,
    EVENT_COUNT_CH1 =         0x26,
    LOW_TH_CH1 =              0x27,
    HYSTERESIS_CH2 =          0x28,
    HIGH_TH_CH2 =             0x29,
    EVENT_COUNT_CH2 =         0x2A,
    LOW_TH_CH2 =              0x2B,
    HYSTERESIS_CH3 =          0x2C,
    HIGH_TH_CH3 =             0x2D,
    EVENT_COUNT_CH3 =         0x2E,
    LOW_TH_CH3 =              0x2F,
    HYSTERESIS_CH4 =          0x30,
    HIGH_TH_CH4 =             0x31,
    EVENT_COUNT_CH4 =         0x32,
    LOW_TH_CH4 =              0x33,
    HYSTERESIS_CH5 =          0x34,
    HIGH_TH_CH5 =             0x35,
    EVENT_COUNT_CH5 =         0x36,
    LOW_TH_CH5 =              0x37,
    HYSTERESIS_CH6 =          0x38,
    HIGH_TH_CH6 =             0x39,
    EVENT_COUNT_CH6 =         0x3A,
    LOW_TH_CH6 =              0x3B,
    HYSTERESIS_CH7 =          0x3C,
    HIGH_TH_CH7 =             0x3D,
    EVENT_COUNT_CH7 =         0x3E,
    LOW_TH_CH7 =              0x3F,
    MAX_CH0_LSB =             0x60,
    MAX_CH0_MSB =             0x61,
    MAX_CH1_LSB =             0x62,
    MAX_CH1_MSB =             0x63,
    MAX_CH2_LSB =             0x64,
    MAX_CH2_MSB =             0x65,
    MAX_CH3_LSB =             0x66,
    MAX_CH3_MSB =             0x67,
    MAX_CH4_LSB =             0x68,
    MAX_CH4_MSB =             0x69,
    MAX_CH5_LSB =             0x6A,
    MAX_CH5_MSB =             0x6B,
    MAX_CH6_LSB =             0x6C,
    MAX_CH6_MSB =             0x6D,
    MAX_CH7_LSB =             0x6E,
    MAX_CH7_MSB =             0x6F,
    MIN_CH0_LSB =             0x80,
    MIN_CH0_MSB =             0x81,
    MIN_CH1_LSB =             0x82,
    MIN_CH1_MSB =             0x83,
    MIN_CH2_LSB =             0x84,
    MIN_CH2_MSB =             0x85,
    MIN_CH3_LSB =             0x86,
    MIN_CH3_MSB =             0x87,
    MIN_CH4_LSB =             0x88,
    MIN_CH4_MSB =             0x89,
    MIN_CH5_LSB =             0x8A,
    MIN_CH5_MSB =             0x8B,
    MIN_CH6_LSB =             0x8C,
    MIN_CH6_MSB =             0x8D,
    MIN_CH7_LSB =             0x8E,
    MIN_CH7_MSB =             0x8F,
    RECENT_CH0_LSB =          0xA0,
    RECENT_CH0_MSB =          0xA1,
    RECENT_CH1_LSB =          0xA2,
    RECENT_CH1_MSB =          0xA3,
    RECENT_CH2_LSB =          0xA4,
    RECENT_CH2_MSB =          0xA5,
    RECENT_CH3_LSB =          0xA6,
    RECENT_CH3_MSB =          0xA7,
    RECENT_CH4_LSB =          0xA8,
    RECENT_CH4_MSB =          0xA9,
    RECENT_CH5_LSB =          0xAA,
    RECENT_CH5_MSB =          0xAB,
    RECENT_CH6_LSB =          0xAC,
    RECENT_CH6_MSB =          0xAD,
    RECENT_CH7_LSB =          0xAE,
    RECENT_CH7_MSB =          0xAF,
    GPO0_TRIG_EVENT_SEL =     0xC3,
    GPO1_TRIG_EVENT_SEL =     0xC5,
    GPO2_TRIG_EVENT_SEL =     0xC7,
    GPO3_TRIG_EVENT_SEL =     0xC9,
    GPO4_TRIG_EVENT_SEL =     0xCB,
    GPO5_TRIG_EVENT_SEL =     0xCD,
    GPO6_TRIG_EVENT_SEL =     0xCF,
    GPO7_TRIG_EVENT_SEL =     0xD1,
    GPO_TRIGGER_CFG =         0xE9,
    GPO_VALUE_TRIG =          0xEB,
};

enum class DataCfg_AppendType : uint8_t {
    NONE = 0x0 << 4,
    ID = 0x1 << 4,
    STATUS = 0x2 << 4,
    BOTH = 0x3 << 4,
};

enum class OsrCfg_Type : uint8_t {
    OSR_NONE = 0x0,
    OSR_2 = 0x1,
    OSR_4 = 0x2,
    OSR_8 = 0x3,
    OSR_16 = 0x4,
    OSR_32 = 0x5,
    OSR_64 = 0x6,
    OSR_128 = 0x7,
};

enum class ConvMode_Type : uint8_t {
    MANUAL = 0x0 << 5,
    AUTONOMOUS = 0x1 << 5,
};

enum class Osc_Type : uint8_t {
    HIGH_SPEED = 0x0 << 4,
    LOW_POWER = 0x1 << 4,
};

enum class SeqMode_Type : uint8_t {
    MANUAL = 0x0 << 0,
    AUTO = 0x1 << 0,
};

/* SYSTEM_STATUS Register Fields */
constexpr uint8_t ADS7138_SYSTEM_STATUS_RSVD =      (0x1 << 7);
constexpr uint8_t ADS7138_SYSTEM_STATUS_SEQ_STATUS =(0x1 << 6);
constexpr uint8_t ADS7138_SYSTEM_I2C_SPEED =        (0x1 << 5);
constexpr uint8_t ADS7138_SYSTEM_OSR_DONE =         (0x1 << 3);
constexpr uint8_t ADS7138_SYSTEM_CRC_ERR_FUSE =     (0x1 << 2);
constexpr uint8_t ADS7138_SYSTEM_CRC_ERR_IN =       (0x1 << 1);
constexpr uint8_t ADS7138_SYSTEM_BOR =              (0x1 << 0);

/* GENERAL_CFG Register Fields */
constexpr uint8_t ADS7138_GENERAL_CFG_CRC_EN =  (0x1 << 6);
constexpr uint8_t ADS7138_GENERAL_CFG_STATS_EN =(0x1 << 5);
constexpr uint8_t ADS7138_GENERAL_CFG_CH_RST =  (0x1 << 2);
constexpr uint8_t ADS7138_GENERAL_CFG_DWC_EN =  (0x1 << 4);
constexpr uint8_t ADS7138_GENERAL_CFG_CNVST =   (0x1 << 3);
constexpr uint8_t ADS7138_GENERAL_CFG_CAL =     (0x1 << 1);
constexpr uint8_t ADS7138_GENERAL_CFG_RST =     (0x1 << 0);

/* DATA_CFG Register Fields */
constexpr uint8_t ADS7138_DATA_CFG_FIX_PAT =        (0x1 << 7);
constexpr uint8_t ADS7138_DATA_CFG_APPEND_STATUS =  (0x3 << 4);

/* OSR_CFG Register Fields */
constexpr uint8_t ADS7138_OSR_CFG_OSR =             (0x7 << 0);

/* OPMODE_CFG Register Fields */
constexpr uint8_t ADS7138_OPMODE_CFG_CONV_ON_ERR =  (0x1 << 7);
constexpr uint8_t ADS7138_OPMODE_CFG_CONV_MODE =    (0x3 << 5);
constexpr uint8_t ADS7138_OPMODE_CFG_OSC_SEL =      (0x1 << 4);
constexpr uint8_t ADS7138_OPMODE_CFG_CLK_DIV =      (0xF << 0);

/* SEQUENCE_CFG Register Fields */
constexpr uint8_t ADS7138_SEQUENCE_CFG_SEQ_START = (0x1 << 4);
constexpr uint8_t ADS7138_SEQUENCE_CFG_SEQ_MODE =  (0x3 << 0);

/* AVDD (VREF) operating range */
#define ADS7138_VREF_MV_MIN	2350
#define ADS7138_VREF_MV_MAX	5500


class ADS7138 {
public:
    ADS7138(I2C_HandleTypeDef *phi2c, uint8_t address);
    HAL_StatusTypeDef Init(); // working
    HAL_StatusTypeDef TestI2C(); // working

    HAL_StatusTypeDef ConfigureStatistics(bool stats_en);
    HAL_StatusTypeDef ConfigureData(bool fix_pattern, DataCfg_AppendType append_type);
    HAL_StatusTypeDef ConfigureOversampling(OsrCfg_Type osr_cfg);
    HAL_StatusTypeDef ConfigureOpmode(bool conv_on_err, ConvMode_Type conv_mode);
    HAL_StatusTypeDef ConfigureOpmode(bool conv_on_err, ConvMode_Type conv_mode, Osc_Type osc_type, uint8_t clk_div);
    HAL_StatusTypeDef ConfigurePinMode(uint8_t pin_mode);
    HAL_StatusTypeDef ConfigureSequence(uint8_t channels);
    HAL_StatusTypeDef ConfigureSequenceAll();
    HAL_StatusTypeDef ConfigureSequenceMode(SeqMode_Type seq_mode);
    HAL_StatusTypeDef ManualSelectChannel(uint8_t channel);

    HAL_StatusTypeDef StartConversion();
    HAL_StatusTypeDef StartSequence();
    HAL_StatusTypeDef StopSequence();

    // 8.4.2 Manual Mode
    HAL_StatusTypeDef ConversionReadManual(uint16_t *buf, uint8_t channel); // working
    // 8.4.3 Auto-Sequence Mode
    HAL_StatusTypeDef ConversionReadAutoSequence(uint16_t *buf, uint8_t len); // not working
    // 8.4.4 Autonomous Mode
    HAL_StatusTypeDef InitAutonomous(uint8_t channels);
    HAL_StatusTypeDef ReadChannel(uint8_t channel, uint16_t *data);
private:
    I2C_HandleTypeDef *_phi2c;
    uint8_t _address;

    ConvMode_Type _conv_mode;
    bool _fix_pattern;
    DataCfg_AppendType _append_type;

    HAL_StatusTypeDef ReadReg(ADS7138_Register reg, uint8_t *data);
    HAL_StatusTypeDef WriteReg(ADS7138_Register reg, uint8_t data);
    HAL_StatusTypeDef SetRegBits(ADS7138_Register reg, uint8_t mask);
    HAL_StatusTypeDef ClearRegBits(ADS7138_Register reg, uint8_t mask);
};