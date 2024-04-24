#ifndef USER_HPP_
#define USER_HPP_

#include "main.h"
#include "logger.hpp"
#include "UI.hpp"
#include "button.hpp"

/* Datamodules */
#include "MitsubaFrames.hpp"
#include "MPPTFrames.hpp"
#include "DriverControls.hpp"
#include "CustomBMSFrames.hpp"

/* Externs */
extern UI ui;
extern Button left_turn_btn;
extern Button mode_btn;
extern Button regen_btn;
extern Button horn_btn;
extern Button mc_btn;
extern Button right_turn_btn;
extern Button cruise_plus_btn;
extern Button cruise_minus_btn;
extern Button ptt_btn;
extern Button pv_btn;

/* Function Prototypes */
void SendMode(SteeringModeTypeDef mode);

#endif
