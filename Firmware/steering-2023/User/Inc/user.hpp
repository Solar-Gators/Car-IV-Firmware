/*
 * user.hpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#ifndef USER_HPP_
#define USER_HPP_

#include "main.h"
#include "SteeringController.hpp"
#include "Mitsuba.hpp"
#include "OrionBMS.hpp"
#include "DataModuleInfo.hpp"
#include "Button.hpp"
#include "CAN.hpp"
#include "ILI9341.hpp"
#include "ILI9341_CMD.hpp"
#include "FrontLights.hpp"
#include "PowerBoard.hpp"
#include "RearLights.hpp"
#include "Mppt.hpp"

SolarGators::DataModules::SteeringController LightsState;
SolarGators::DataModules::MitsubaRequest McReq(SolarGators::DataModuleInfo::MOTORTX_RL_MSG_ID);
SolarGators::DataModules::OrionBMSRx0 BMS_Rx_0(SolarGators::DataModuleInfo::BMS_RX0_MSG_ID, 0);
SolarGators::DataModules::OrionBMSRx1 BMS_Rx_1(SolarGators::DataModuleInfo::BMS_RX1_MSG_ID, 0);
SolarGators::DataModules::OrionBMSRx2 BMS_Rx_2(SolarGators::DataModuleInfo::BMS_RX2_MSG_ID, 0);
SolarGators::DataModules::OrionBMSRx4 BMS_Rx_4(SolarGators::DataModuleInfo::BMS_RX4_MSG_ID, 0);
SolarGators::DataModules::MitsubaRx0 Motor_Rx_0(SolarGators::DataModuleInfo::MOTORRX0_RL_MSG_ID, 0x04);
SolarGators::DataModules::MitsubaRx2 Motor_Rx_2(SolarGators::DataModuleInfo::MOTORRX2_RL_MSG_ID, 0x04);
SolarGators::DataModules::PowerBoard PowerBoard;
SolarGators::DataModules::FrontLights FLights;
SolarGators::DataModules::RearLights RLights;
SolarGators::DataModules::Mpptx1 MPPT0_Rx_1(SolarGators::DataModuleInfo::MPPT0_RX1_MSG_ID);
SolarGators::DataModules::Mpptx1 MPPT1_Rx_1(SolarGators::DataModuleInfo::MPPT1_RX1_MSG_ID);
SolarGators::DataModules::Mpptx1 MPPT2_Rx_1(SolarGators::DataModuleInfo::MPPT2_RX1_MSG_ID);

// Forward Declares
extern CAN_HandleTypeDef hcan1;
// extern TIM_HandleTypeDef htim2;

// CAN Driver
SolarGators::Drivers::CANDriver CANController(&hcan1, 0);

// Buttons
// Left Side
SolarGators::Drivers::Button left_turn    ("Left Turn", BTN4_GPIO_Port, BTN4_Pin);
SolarGators::Drivers::Button headlights   ("Headlights", BTN3_GPIO_Port, BTN3_Pin);
//SolarGators::Drivers::Button eco          ("Eco Enable", BTN8_GPIO_Port, BTN8_Pin); // (regen)
//SolarGators::Drivers::Button horn         ("Horn", BTN7_GPIO_Port, BTN7_Pin);
SolarGators::Drivers::Button reverse      ("Reverse", BTN1_GPIO_Port, BTN1_Pin);

// Right Side
SolarGators::Drivers::Button right_turn   ("Right Turn", BTN9_GPIO_Port, BTN9_Pin);
SolarGators::Drivers::Button hazards      ("Hazards", BTN0_GPIO_Port, BTN0_Pin);
SolarGators::Drivers::Button cruise       ("Cruise", BTN2_GPIO_Port, BTN2_Pin);
SolarGators::Drivers::Button cruise_plus  ("Cruise Plus", BTN6_GPIO_Port, BTN6_Pin);
SolarGators::Drivers::Button cruise_minus ("Cruise Minus", BTN5_GPIO_Port, BTN5_Pin);

#endif
