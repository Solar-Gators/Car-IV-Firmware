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

#define SOC_THRESHOLD                   20.0f
#define BATT_VOLTAGE_THRESHOLD          80.0f
#define AUX_VOLTAGE_THRESHOLD           11.2f
#define TEMP_THRESHOLD                  42u

#define AUX_HIGH_CURRENT_FAULT          "       12V High Current"
#define WEAK_CELL_FAULT                 "        Weak Cell Fault"

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
    HAL_StatusTypeDef DeactivateLeftTurn();
    HAL_StatusTypeDef DeactivateRightTurn();
    HAL_StatusTypeDef ToggleLeftTurn();
    HAL_StatusTypeDef ToggleRightTurn();
    HAL_StatusTypeDef UpdatePVStatus(uint16_t color);
    HAL_StatusTypeDef UpdateBMSStatus(uint16_t color);
    HAL_StatusTypeDef UpdateMCStatus(uint16_t color);
    HAL_StatusTypeDef IncrementTime();
    HAL_StatusTypeDef ResetTime();
    HAL_StatusTypeDef UpdateLaps(uint32_t value);
    HAL_StatusTypeDef UpdateSOC(float value);
    HAL_StatusTypeDef UpdateBattV(float value);
    HAL_StatusTypeDef UpdateAuxV(float value);
    HAL_StatusTypeDef UpdateTemp(float value);
    HAL_StatusTypeDef UpdateEfficiency(uint32_t value);
    HAL_StatusTypeDef UpdateNetPower(uint32_t value);
    HAL_StatusTypeDef UpdateSolarPower(uint32_t value);
    HAL_StatusTypeDef UpdateMotorPower(uint32_t value);
    HAL_StatusTypeDef DisplayError1(const char* str);
    HAL_StatusTypeDef DisplayError2(const char* str);
protected:
    ILI9341* display_;
    bool left_turn_ = false;
    bool right_turn_ = false;
    uint32_t minutes_;
    uint32_t seconds_;
private:
    HAL_StatusTypeDef DrawTime();
};


#endif  /* SG_UI_HPP_ */
