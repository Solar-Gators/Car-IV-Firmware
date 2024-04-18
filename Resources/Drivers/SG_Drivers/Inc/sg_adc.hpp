/*
 * ADS7138 ADC I2C Driver
 *
 * Created on: 1/12/2024
 * Authors: Anthony Kfoury, Braden Azis
 */
#ifndef ADS7138_I2C_DRIVER_H
#define ADS7138_I2C_DRIVER_H

#define ADS7138_I2C_ADDR 0x10

#include "stm32l4xx_hal.h"
#include "main.h"

typedef class{
    public:
    uint16_t maxchannelData[8];
    float maxfchannelData[8];
    uint16_t minchannelData[8];
    float minfchannelData[8];
    I2C_HandleTypeDef *i2c;
}ADCDevice;

//configuration
#define GENERAL_CFG 0x1
#define DATA_CFG 0x2
#define OSR_CFG 0x3
#define OPMODE_CFG 0x4
#define PIN_CFG 0x5
#define GPIO_CFG 0x7
#define GPO_DRIVE_CFG 0x9
#define SEQUENCE_CFG 0x10
#define ALERT_PIN_CFG 0x17


// Max Channels
 

#define MAX_CH0_LSB 0x60
#define MAX_CH0_MSB 0x61
 
#define MAX_CH1_LSB 0x62
#define MAX_CH1_MSB 0x63
 
#define MAX_CH2_LSB 0x64
#define MAX_CH2_MSB 0x65
 
#define MAX_CH3_LSB 0x66
#define MAX_CH3_MSB 0x67
 
#define MAX_CH4_LSB 0x68
#define MAX_CH4_MSB 0x69
 
#define MAX_CH5_LSB 0x6A
#define MAX_CH5_MSB 0x6B
 
#define MAX_CH6_LSB 0x6C
#define MAX_CH6_MSB 0x6D
 
#define MAX_CH7_LSB 0x6E
#define MAX_CH7_MSB 0x6F
 
//Min Channels
 
#define MIN_CH0_LSB 0x80
#define MIN_CH0_MSB 0x81
 
#define MIN_CH1_LSB 0x82
#define MIN_CH1_MSB 0x83
 
#define MIN_CH2_LSB 0x84
#define MIN_CH2_MSB 0x85
 
#define MIN_CH3_LSB 0x86
#define MIN_CH3_MSB 0x87
 
#define MIN_CH4_LSB 0x88
#define MIN_CH4_MSB 0x89
 
#define MIN_CH5_LSB 0x8A
#define MIN_CH5_MSB 0x8B
 
#define MIN_CH6_LSB 0x8C
#define MIN_CH6_MSB 0x8D
 
#define MIN_CH7_LSB 0x8E
#define MIN_CH7_MSB 0x8F
 
 
//Recent Channels
 
#define RECENT_CH0_LSB 0xA0
#define RECENT_CH0_MSB 0xA1
 
#define RECENT_CH1_LSB 0xA2
#define RECENT_CH1_MSB 0xA3
 
#define RECENT_CH2_LSB 0xA4
#define RECENT_CH2_MSB 0xA5
 
#define RECENT_CH3_LSB 0xA6
#define RECENT_CH3_MSB 0xA7
 
#define RECENT_CH4_LSB 0xA8
#define RECENT_CH4_MSB 0xA9
 
#define RECENT_CH5_LSB 0xAA
#define RECENT_CH5_MSB 0xAB
 
#define RECENT_CH6_LSB 0xAC
#define RECENT_CH6_MSB 0xAD
 
#define RECENT_CH7_LSB 0xAE
#define RECENT_CH7_MSB 0xAF
 
 
//Trigger Events
 
#define GPO0_TRIG_EVENT_SEL 0xC3
#define GPO1_TRIG_EVENT_SEL 0xC5
#define GPO2_TRIG_EVENT_SEL 0xC7
#define GPO3_TRIG_EVENT_SEL 0xC9
#define GPO4_TRIG_EVENT_SEL 0xCB
#define GPO5_TRIG_EVENT_SEL 0xCD
#define GPO6_TRIG_EVENT_SEL 0xCF
#define GPO7_TRIG_EVENT_SEL 0xD1
#define GPO_TRIGGER_CFG 0xE9
#define GPO_VALUE_TRIG 0xE8

// functions
uint8_t ADS7138_Init(ADCDevice *adc, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef ADC_UpdateAllMinChannel(ADCDevice *adc);
HAL_StatusTypeDef ADC_UpdateAllMaxChannel(ADCDevice *adc);
HAL_StatusTypeDef ADC_UpdateMinChannel(ADCDevice *adc, uint8_t channelNumber);
HAL_StatusTypeDef ADC_UpdateMaxChannel(ADCDevice *adc, uint8_t channelNumber);
HAL_StatusTypeDef ADC_ResetChannels(ADCDevice *adc);
HAL_StatusTypeDef ADC_ReadRegister(ADCDevice *adc, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ADC_WriteRegister(ADCDevice *adc, uint8_t reg, uint8_t *data);
#endif