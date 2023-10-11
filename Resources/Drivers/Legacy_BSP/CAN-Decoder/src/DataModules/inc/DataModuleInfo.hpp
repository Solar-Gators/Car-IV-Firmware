/*
 * DataModuleInfo.hpp
 *
 *  Created on: Jan 14, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_DATAMODULEINFO_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_DATAMODULEINFO_HPP_


namespace SolarGators::DataModuleInfo
{
// ---- Addresses ---- //

// BMS
static constexpr uint32_t BMS_RX0_MSG_ID = 0x6B0;
static constexpr uint32_t BMS_RX1_MSG_ID = 0x6B1;
static constexpr uint32_t BMS_RX2_MSG_ID = 0x6B2;
static constexpr uint32_t BMS_RX3_MSG_ID = 0x6B3;
static constexpr uint32_t BMS_RX4_MSG_ID = 0x6B4;
static constexpr uint32_t BMS_RX5_MSG_ID = 0x6B5;

// Mitsuba
//TX Messages
static constexpr uint32_t MOTORTX_RL_MSG_ID = 0x08F89540;
//RX Messages
static constexpr uint32_t MOTORRX0_RL_MSG_ID = 0x08850225;
static constexpr uint32_t MOTORRX1_RL_MSG_ID = 0x08950225;
static constexpr uint32_t MOTORRX2_RL_MSG_ID = 0x08A50225;

// MPPT
static constexpr uint16_t MPPT0_ID = 0x600;
static constexpr uint16_t MPPT0_RX0_MSG_ID = MPPT0_ID + 0;
static constexpr uint16_t MPPT0_RX1_MSG_ID = MPPT0_ID + 1;
static constexpr uint16_t MPPT0_RX2_MSG_ID = MPPT0_ID + 2;
static constexpr uint16_t MPPT0_RX3_MSG_ID = MPPT0_ID + 3;
static constexpr uint16_t MPPT0_RX4_MSG_ID = MPPT0_ID + 4;
static constexpr uint16_t MPPT0_RX5_MSG_ID = MPPT0_ID + 5;
static constexpr uint16_t MPPT0_RX6_MSG_ID = MPPT0_ID + 6;

static constexpr uint16_t MPPT1_ID = 0x610;
static constexpr uint16_t MPPT1_RX0_MSG_ID = MPPT1_ID + 0;
static constexpr uint16_t MPPT1_RX1_MSG_ID = MPPT1_ID + 1;
static constexpr uint16_t MPPT1_RX2_MSG_ID = MPPT1_ID + 2;
static constexpr uint16_t MPPT1_RX3_MSG_ID = MPPT1_ID + 3;
static constexpr uint16_t MPPT1_RX4_MSG_ID = MPPT1_ID + 4;
static constexpr uint16_t MPPT1_RX5_MSG_ID = MPPT1_ID + 5;
static constexpr uint16_t MPPT1_RX6_MSG_ID = MPPT1_ID + 6;

static constexpr uint16_t MPPT2_ID = 0x620;
static constexpr uint16_t MPPT2_RX0_MSG_ID = MPPT2_ID + 0;
static constexpr uint16_t MPPT2_RX1_MSG_ID = MPPT2_ID + 1;
static constexpr uint16_t MPPT2_RX2_MSG_ID = MPPT2_ID + 2;
static constexpr uint16_t MPPT2_RX3_MSG_ID = MPPT2_ID + 3;
static constexpr uint16_t MPPT2_RX4_MSG_ID = MPPT2_ID + 4;
static constexpr uint16_t MPPT2_RX5_MSG_ID = MPPT2_ID + 5;
static constexpr uint16_t MPPT2_RX6_MSG_ID = MPPT2_ID + 6;

// Steering Wheel
static constexpr uint16_t STEERING_ID = 2048;

// Front Lights
static constexpr uint16_t FRONT_LIGHTS_ID = 2049;

// Rear Lights
static constexpr uint16_t REAR_LIGHTS_ID = 2050;

// power board
static constexpr uint16_t POWER_BOARD_ID = 2051;

}


#endif /* SOLARGATORSBSP_DATAMODULES_INC_DATAMODULEINFO_HPP_ */
