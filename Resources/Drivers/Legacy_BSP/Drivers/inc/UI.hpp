/*
 * UI.h
 *
 *  Created on: Jan 12, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DRIVERS_INC_UI_HPP_
#define SOLARGATORSBSP_DRIVERS_INC_UI_HPP_

#include "main.h"
#include "ILI9341.hpp"

// ETL Stuff
#include "etl/array.h"
#include "etl/string.h"
#include "etl/to_string.h"
#include "etl/string_utilities.h"
#include "etl/format_spec.h"

// RTOS Stuff
#include <cmsis_os.h>

// CAN messages
#include "Mitsuba.hpp"
#include "OrionBMS.hpp"

namespace SolarGators {
namespace Drivers {

// Minor UI Element Struct
class InfoSquare {
public:
  etl::string<5> title;
  uint16_t x;
  uint16_t y;
  static constexpr uint16_t TextPaddX  = 10;
  static constexpr uint16_t TitlePaddY = 7;
  static constexpr uint16_t DataPaddY  = 4;
  static constexpr uint16_t DataSqW    = 80;
  static constexpr uint16_t DataSqH    = 50;
  static constexpr uint16_t TextWidth  = 6;
  static constexpr uint16_t TextHeight = 8;
  static constexpr uint8_t TextSize = 2;
  void Draw(ILI9341& disp);
  void UpdateValue(ILI9341& disp, etl::string<5>& val);
};

class UI {
public:
  UI(ILI9341& display);
  ~UI();
  void UpdateBMSTrip(SolarGators::DataModules::OrionBMSRx4* bmsCodes);
  void UpdateMitsubaTrip(SolarGators::DataModules::MitsubaRx2* mitsubaCodes);
  void ClearIndicators();
  void SetLeftTurn();
  void SetRightTurn();
  void SetHazards();
  void SetHeadlights();
  void ClearHeadlights();
  void UpdateSupBat(float voltage);
  void SetEco();
  void ClearEco();
  void SetReverse();
  void ClearReverse();
  void SetCruise();
  void ClearCruise();
  void SetExternalTrip();
  void Update();
  ILI9341& disp;
  void UpdateSquare(uint8_t num, etl::string<5>& val);
  void UpdateSpeed(float speed, uint8_t regen);
  static constexpr uint32_t WHITE   = 0xFFFF;
  static constexpr uint32_t BLACK   = 0x0000;
  static constexpr uint32_t BLUE    = 0x0197;
  static constexpr uint32_t RED     = 0xF800;
  static constexpr uint32_t MAGENTA = 0xF81F;
  static constexpr uint32_t GREEN   = 0x07E0;
  static constexpr uint16_t DARK_GREEN = 0x8F00;
  static constexpr uint32_t CYAN    = 0x7FFF;
  static constexpr uint32_t YELLOW  = 0xFFE0;
  static constexpr uint32_t GRAY    = 0x2104;
  static constexpr uint32_t PURPLE  = 0xF11F;
  static constexpr uint32_t ORANGE  = 0xFD20;
  static constexpr uint32_t PINK    = 0xfdba;
  static constexpr uint32_t OLIVE   = 0xdfe4;
  static constexpr uint16_t color_neutral_ = WHITE;
  static constexpr uint16_t color_ok_ = GREEN;
  static constexpr uint16_t color_fail_ = RED;
private:
  etl::array<InfoSquare, 4> first_row_;
  etl::array<InfoSquare, 2> second_row_;
  void DrawSpeed();
  void DrawTripCodes();
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DRIVERS_INC_UI_HPP_ */
