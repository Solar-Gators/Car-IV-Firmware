#include "UI.hpp"

#include <etl/string.h>
#include "etl/to_string.h"
#include "etl/string_utilities.h"
#include "etl/format_spec.h"

UI::UI(ILI9341* display) {
    this->display_ = display;
    this->left_turn_ = false;
    this->right_turn_ = false;
    this->minutes_ = 0;
    this->seconds_ = 0;
}

UI::~UI() {
    // TODO Auto-generated destructor stub
}

HAL_StatusTypeDef UI::Init() {
    // Initialize display
    display_->Init();
    display_->ClearScreen(RGB565_BLACK);
    display_->SetRotation(1);

    // Draw border rectangles
    display_->DrawRect(0, 0, 220, 60, RGB565_WHITE);
	display_->DrawRect(220, 0, 100, 60, RGB565_WHITE);
	display_->DrawRect(0, 60, 320, 30, RGB565_WHITE);
	display_->DrawRect(0, 90, 320, 150, RGB565_WHITE);

    // Draw reset speed
    UpdateSpeed(0.0);

    // Draw reset time
    ResetTime();

    // Draw info labels
    display_->SetTextSize(2);
    display_->DrawText(10, 142, "Laps:", RGB565_WHITE);
	display_->DrawText(10, 161, "SOC:", RGB565_WHITE);
	display_->DrawText(10, 180, "Batt:", RGB565_WHITE);
	display_->DrawText(10, 199, "Aux:", RGB565_WHITE);
	display_->DrawText(10, 218, "Temp:", RGB565_WHITE);
    display_->DrawText(232, 135, "W Net", RGB565_YELLOW);
	display_->DrawText(232, 163, "W MPPT1", RGB565_CYAN);
	display_->DrawText(232, 191, "W MPPT2", RGB565_PINK);
	display_->DrawText(232, 219, "W MPPT3", RGB565_DARK_GREEN);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateSpeed(float speed) {
    etl::string<8> str_speed;
    etl::to_string(speed, str_speed, etl::format_spec().precision(1).width(4).fill('\0'), false);

    str_speed.append(" MPH");

    display_->SetTextSize(4);
    display_->DrawText(16, 15, str_speed.c_str(), RGB565_YELLOW);
    
    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateMode(SteeringModeTypeDef mode) {
    display_->SetTextSize(3);

    switch (mode) {
        case MODE_ECO:
            display_->DrawText(242, 20, "ECO", RGB565_GREEN);
            break;
        case MODE_PWR:
            display_->DrawText(242, 20, "PWR", RGB565_BLUE);
            break;
        case MODE_CRS:
            display_->DrawText(242, 20, "CRS", RGB565_PURPLE);
            break;
        case MODE_REV:
            display_->DrawText(242, 20, "REV", RGB565_RED);
            break;
        default:
            break;
    }

    return HAL_OK;
}

HAL_StatusTypeDef UI::DeactivateLeftTurn() {
    left_turn_ = false;

    display_->SetTextSize(2);
    display_->DrawText(20, 67, "<", RGB565_BLACK);

    return HAL_OK;
}

HAL_StatusTypeDef UI::DeactivateRightTurn() {
    right_turn_ = false;

    display_->SetTextSize(2);
    display_->DrawText(285, 67, ">", RGB565_BLACK);

    return HAL_OK;
}

HAL_StatusTypeDef UI::ToggleLeftTurn() {
    left_turn_ = !left_turn_;

    display_->SetTextSize(2);
    if (left_turn_) {
        display_->DrawText(20, 67, "<", RGB565_GREEN);
    } else {
        display_->DrawText(20, 67, "<", RGB565_BLACK);
    }

    return HAL_OK;
}

HAL_StatusTypeDef UI::ToggleRightTurn() {
    right_turn_ = !right_turn_;

    display_->SetTextSize(2);
    if (right_turn_) {
        display_->DrawText(285, 67, ">", RGB565_GREEN);
    } else {
        display_->DrawText(285, 67, ">", RGB565_BLACK);
    }

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdatePVStatus(uint16_t color) {
    display_->SetTextSize(2);
    display_->DrawText(80, 67, "PV", color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateBMSStatus(uint16_t color) {
    display_->SetTextSize(2);
    display_->DrawText(140, 67, "BMS", color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateMCStatus(uint16_t color) {
    display_->SetTextSize(2);
    display_->DrawText(210, 67, "MC", color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::IncrementTime() {
    seconds_++;
    if (seconds_ == 60) {
        seconds_ = 0;
        minutes_++;
    }
    if (minutes_ == 99) {
        minutes_ = 0;
    }

    return DrawTime();
}

HAL_StatusTypeDef UI::ResetTime() {
    minutes_ = 0;
    seconds_ = 0;

    return DrawTime();
}

HAL_StatusTypeDef UI::UpdateLaps(uint32_t value) {
    etl::string<3> str_value;
    etl::to_string(value, str_value, etl::format_spec().width(3).fill('0'), false);

    display_->SetTextSize(2);
    display_->DrawText(80, 142, str_value.c_str(), RGB565_WHITE);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateSOC(float value) {
    etl::string<5> str_value;
    etl::to_string(value, str_value, etl::format_spec().precision(1).width(3).fill(' '), false);

    uint16_t color = RGB565_WHITE;
    if (value < SOC_THRESHOLD)
        color = RGB565_RED;

    display_->SetTextSize(2);
    display_->DrawText(80, 161, str_value.c_str(), color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateBattV(float value) {
    etl::string<5> str_value;
    etl::to_string(value, str_value, etl::format_spec().precision(1).width(3).fill(' '), false);

    uint16_t color = RGB565_WHITE;
    if (value < BATT_VOLTAGE_THRESHOLD)
        color = RGB565_RED;

    display_->SetTextSize(2);
    display_->DrawText(80, 180, str_value.c_str(), color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateAuxV(float value) {
    etl::string<5> str_value;
    etl::to_string(value, str_value, etl::format_spec().precision(2).width(2).fill(' '), false);

    uint16_t color = RGB565_WHITE;
    if (value < AUX_VOLTAGE_THRESHOLD)
        color = RGB565_RED;

    display_->SetTextSize(2);
    display_->DrawText(80, 199, str_value.c_str(), color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateTemp(float value) {
    etl::string<4> str_value;
    etl::to_string(value, str_value, etl::format_spec().precision(1).width(2).fill(' '), false);

    uint16_t color = RGB565_WHITE;
    if (value > TEMP_THRESHOLD)
        color = RGB565_RED;

    display_->SetTextSize(2);
    display_->DrawText(80, 218, str_value.c_str(), color);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateEfficiency(uint32_t value) {
    etl::string<4> str_value;
    etl::to_string(value, str_value, etl::format_spec().width(4).right().fill(' '), false);

    display_->SetTextSize(3);
    display_->DrawText(160, 128, str_value.c_str(), RGB565_YELLOW);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateNetPower(int32_t value) {
    etl::string<4> str_value;
    etl::to_string(value, str_value, etl::format_spec().width(4).right().fill(' '), false);

    display_->SetTextSize(3);
    display_->DrawText(160, 156, str_value.c_str(), RGB565_CYAN);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateSolarPower(int32_t value) {
    etl::string<4> str_value;
    etl::to_string(value, str_value, etl::format_spec().width(4).right().fill(' '), false);

    display_->SetTextSize(3);
    display_->DrawText(160, 184, str_value.c_str(), RGB565_PINK);

    return HAL_OK;
}

HAL_StatusTypeDef UI::UpdateMotorPower(int32_t value) {
    etl::string<4> str_value;
    etl::to_string(value, str_value, etl::format_spec().width(4).right().fill(' '), false);

    display_->SetTextSize(3);
    display_->DrawText(160, 212, str_value.c_str(), RGB565_DARK_GREEN);

    return HAL_OK;
}

HAL_StatusTypeDef UI::DrawTime() {
    etl::string<2> str_minutes;
    etl::string<2> str_seconds;

    etl::to_string(minutes_, str_minutes, etl::format_spec().width(2).fill('0'), false);
    etl::to_string(seconds_, str_seconds, etl::format_spec().width(2).fill('0'), false);

    display_->SetTextSize(4);
    display_->DrawText(10, 103, str_minutes.c_str(), RGB565_CYAN);
    display_->DrawText(58, 103, ":", RGB565_CYAN);
    display_->DrawText(82, 103, str_seconds.c_str(), RGB565_CYAN);

    return HAL_OK;
}

HAL_StatusTypeDef UI::DisplayError1(const char* str) {
    display_->SetTextSize(1);
    display_->DrawText(160, 100, str, RGB565_RED);

    return HAL_OK;
}

HAL_StatusTypeDef UI::DisplayError2(const char* str) {
    display_->SetTextSize(1);
    display_->DrawText(160, 112, str, RGB565_RED);

    return HAL_OK;
}