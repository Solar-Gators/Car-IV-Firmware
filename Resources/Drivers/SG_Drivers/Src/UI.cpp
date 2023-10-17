#include "UI.hpp"

#include <etl/string.h>
#include "etl/to_string.h"
#include "etl/string_utilities.h"
#include "etl/format_spec.h"

UI::UI(ILI9341* display) {
    this->display_ = display;
}

UI::~UI() {
    // TODO Auto-generated destructor stub
}

HAL_StatusTypeDef UI::Init() {
    display_->Init();

    display_->ClearScreen(RGB565_BLACK);

    display_->SetRotation(1);

    // Draw border rectangles
    display_->DrawRect(0, 0, 220, 60, RGB565_WHITE);
	display_->DrawRect(220, 0, 100, 60, RGB565_WHITE);
	display_->DrawRect(0, 60, 320, 30, RGB565_WHITE);
	display_->DrawRect(0, 90, 320, 150, RGB565_WHITE);

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

HAL_StatusTypeDef UI::UpdatePVStatus(bool status) {
    display_->SetTextSize(2);
    if (status) {
        display_->DrawText(272, 67, "PV", RGB565_GREEN);
    } else {
        display_->DrawText(272, 67, "PV", RGB565_RED);
    }

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

    etl::string<4> str_minutes;
    etl::string<4> str_seconds;

    etl::to_string(minutes_, str_minutes, etl::format_spec().width(2).fill('0'), false);
    etl::to_string(seconds_, str_seconds, etl::format_spec().width(2).fill('0'), false);

    display_->SetTextSize(4);
	display_->DrawText(10, 103, str_minutes.c_str(), RGB565_CYAN);
    display_->DrawText(58, 103, ":", RGB565_CYAN);
	display_->DrawText(82, 103, str_seconds.c_str(), RGB565_CYAN);

    return HAL_OK;
}