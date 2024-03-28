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
#define ADS7138_OPCODE_READ		0x10
#define ADS7138_OPCODE_WRITE	0x08

/* ADS7138 registers */
#define ADS7138_REG_SYSTEM_STATUS	0x0
#define ADS7138_REG_GENERAL_CFG		0x1
#define ADS7138_REG_DATA_CFG		0x2
#define ADS7138_REG_OSR_CFG			0x3
#define ADS7138_REG_OPMODE_CFG		0x4
#define ADS7138_REG_PIN_CFG			0x5
#define ADS7138_REG_GPIO_CFG        0x7
#define ADS7138_REG_GPIO_DRIVE_CFG  0x9
#define ADS7138_REG_GPO_VALUE       0xB
#define ADS7138_REG_GPI_VALUE       0xD
#define ADS7138_REG_SEQUENCE_CFG    0x10
#define ADS7138_REG_CHANNEL_SEL     0x11
#define ADS7138_AUTO_SEQ_CH_SEL     0x12
#define ADS7138_REG_ALERT_CH_SEL    0x14
#define ADS7138_REG_ALERT_MAP       0x16
#define ADS7138_REG_ALERT_PIN_CFG   0x17
#define ADS7138_REG_EVENT_FLAG      0x18
#define ADS7138_REG_EVENT_HIGH_FLAG 0x1A
#define ADS7138_REG_EVENT_LOW_FLAG  0x1C
#define ADS7138_REG_EVENT_RGN_FLAG  0x1E


#define ADS7138_REG_RECENT_CH0_LSB  0xA0
#define ADS7138_REG_RECENT_CH0_MSB  0xA1

/* GENERAL_CFG Register Fields */
#define ADS7138_GENERAL_CFG_RST		0x1

/* DATA_CFG Register Fields */
#define ADS7138_DATA_CFG_FIX_PAT 	0x80
#define ADS7138_DATA_CFG_CH_ID	 	0x10

/* OSR_CFG Register Fields */
#define ADS7138_OSR_CFG_NO_AVG	 	0x0
#define ADS7138_OSR_CFG_2	 		0x1

/* AVDD (VREF) operating range */
#define ADS7138_VREF_MV_MIN	2350
#define ADS7138_VREF_MV_MAX	5500


class ADS7138 {
public:
    ADS7138(I2C_HandleTypeDef *phi2c, uint8_t address);
    HAL_StatusTypeDef Init();
    HAL_StatusTypeDef TestI2C();
private:
    I2C_HandleTypeDef *_phi2c;
    uint8_t _address;
    HAL_StatusTypeDef ReadReg(uint8_t reg, uint8_t *data);
    HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t data);
};