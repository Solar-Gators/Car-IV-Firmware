#pragma once

#include "main.h"
#include "logger.hpp"

/* ADS7138 registers */
enum class TMC4671_Register : uint8_t {
    CHIPINFO_DATA =             0x0,
    CHIPINFO_ADDR =             0x1,
    ADC_RAW_DATA =              0x2,
    ADC_RAW_ADDR =              0x3,
    dsADC_MCFG_B_MCFG_A =       0x4,
    dsADC_MCLK_A =              0x5,
    dsADC_MCLK_B =              0x6,
    dsADC_MDEC_B_MDEC_A =       0x7,
    ADC_I1_SCALE_OFFSET =       0x8,
    ADC_I0_SCALE_OFFSET =       0x9,
    ADC_I_SELECT =              0xA,
    ADC_I1_I0_EXT =             0xB,
    DS_ANALOG_INPUT_STAGE_CFG = 0xC,
    AENC_0_SCALE_OFFSET =       0xD,
    AENC_1_SCALE_OFFSET =       0xE,
    AENC_2_SCALE_OFFSET =       0xF,
    AENC_SELECT =               0x11,
    ADC_IWY_IUX =               0x12,
    ADC_IV =                    0x13,
    AENC_WY_UX =                0x15,
    AENC_VN  =                  0x16,
    PWM_POLARITIES =            0x17,
    PWM_MAXCNT =                0x18,
    PWM_BBM_H_BBM_L =           0x19,
    PWM_SV_CHOP =               0x1A,
    MOTOR_TYPE_N_POLE_PAIRS =   0x1B,
    PHI_E_EXT =                 0x1C,
    OPENLOOP_MODE =             0x1F,
    OPENLOOP_ACCELERATION =     0x20,
    OPENLOOP_VELOCITY_TARGET =  0x21,
    OPENLOOP_VELOCITY_ACTUAL =  0x22,
    OPENLOOP_PHI =              0x23,
    UQ_UD_EXT =                 0x24,
    ABN_DECODER_MODE =          0x25,
    ABN_DECODER_PPR =           0x26,
    ABN_DECODER_COUNT =         0x27,
    ABN_DECODER_COUNT_N  =      0x28,
    ABN_DECODER_PHI_E_PHI_M_OFFSET = 0x29,
    ABN_DECODER_PHI_E_PHI_M =   0x2A,
    HALL_MODE =                 0x2C,
    HALL_POSITION_060_000 =     0x34,
    HALL_POSITION_180_120 =     0x35,
    HALL_POSITION_300_240 =     0x36,
    HALL_PHI_E_PHI_M_OFFSET =   0x37,
    HALL_DPHI_MAX =             0x38,
    HALL_PHI_E_INTERPOLATED_PHI_E  =             0x39,
    HALL_PHI_M =         0x3A,
    AENC_DECODER_MODE =              0x3B,
    AENC_DECODER_N_THRESHOLD =          0x3C,
    AENC_DECODER_PHI_A_RAW  =             0x3D,
    AENC_DECODER_PHI_A_OFFSET =         0x3E,
    AENC_DECODER_PHI_A =              0x3F,
    AENC_DECODER_PPR =             0x60,
    
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

class TMC4671 {
public:
    TMC4671(SPI_HandleTypeDef *phspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    
private:
    SPI_HandleTypeDef *hspi_;
    GPIO_TypeDef *cs_port_;
    uint16_t cs_pin_;

    HAL_StatusTypeDef ReadReg8(TMC4671_Register reg, uint8_t *data);
    HAL_StatusTypeDef ReadReg16(TMC4671_Register reg, uint16_t *data);
    HAL_StatusTypeDef ReadReg32(TMC4671_Register reg, uint32_t *data);

    HAL_StatusTypeDef WriteReg8(TMC4671_Register reg, uint8_t data);
    HAL_StatusTypeDef WriteReg16(TMC4671_Register reg, uint16_t data);
    HAL_StatusTypeDef WriteReg32(TMC4671_Register reg, uint32_t data);
};