/*
 * UI.hpp
 *
 *  Created on: Oct 16, 2023
 *      Author: Matthew Shen
 */

#ifndef SG_UI_HPP_
#define SG_UI_HPP_

#include "main.h"
#include "ILI9341.hpp"

typedef enum {
    MODE_ECO,
    MODE_PWR,
    MODE_CRS,
    MODE_REV,
} SteeringModeTypeDef;

class UI {
public:
    UI(ILI9341* display);
    virtual ~UI();
    HAL_StatusTypeDef Init();
    HAL_StatusTypeDef UpdateSpeed(float speed);
    HAL_StatusTypeDef UpdateMode(SteeringModeTypeDef mode);
    HAL_StatusTypeDef ToggleLeftTurn();
    HAL_StatusTypeDef ToggleRightTurn();
    HAL_StatusTypeDef UpdatePVStatus(bool status);
    HAL_StatusTypeDef UpdateBMSStatus(bool status);
    HAL_StatusTypeDef UpdateMCStatus(bool status);
    HAL_StatusTypeDef IncrementTime();
    HAL_StatusTypeDef ResetTime();
    HAL_StatusTypeDef UpdateLaps(uint32_t value);
    HAL_StatusTypeDef UpdateSOC(float value);
    HAL_StatusTypeDef UpdateBattV(float value);
    HAL_StatusTypeDef UpdateAuxV(float value);
    HAL_StatusTypeDef UpdateTemp(float value);
    HAL_StatusTypeDef UpdateEfficiency(uint32_t value);
    HAL_StatusTypeDef UpdateNetPower(float value);
    HAL_StatusTypeDef UpdateSolarPower(float value);
    HAL_StatusTypeDef UpdateMotorPower(float value);
    // TODO: Add soft reset for display
protected:
    ILI9341* display_;
    bool left_turn_ = false;
    bool right_turn_ = false;
    uint32_t minutes_;
    uint32_t seconds_;
};


#endif  /* SG_UI_HPP_ */
