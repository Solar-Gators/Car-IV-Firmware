/*
 * UI.cpp
 *
 *  Created on: Jan 12, 2022
 *      Author: John Carr
 */

#include <UI.hpp>

namespace SolarGators {
namespace Drivers {

void InfoSquare::Draw(ILI9341& disp)
{
  disp.SetTextSize(TextSize);
  // Draw the Border
  disp.DrawRect(x, y, DataSqW, DataSqH, UI::color_neutral_);
  // Draw the Title
  disp.DrawText(x+TextPaddX, y+TitlePaddY, title.c_str(), UI::color_neutral_);
  etl::string<5> tmp(" N/A ");
  UpdateValue(disp, tmp);
}

void InfoSquare::UpdateValue(ILI9341& disp, etl::string<5>& val)
{
  disp.SetTextSize(TextSize);
  disp.DrawText(x+TextPaddX, TextHeight*TextSize+TitlePaddY+DataPaddY+y, val.c_str(), UI::color_neutral_);
}

UI::UI(ILI9341& display):disp(display)
{
  // Set Screen Orientation
  disp.SetRotation(1);
  // Set Background Color
  disp.ClearScreen(0x0000);
  // Setup info squares
  etl::string<5> titles[] = {"Temp"," SOC ","Power","Curnt"};
  for (size_t i = 0; i < first_row_.size(); ++i)
  {
    first_row_[i].title = titles[i];
    first_row_[i].x = i*InfoSquare::DataSqW;
    first_row_[i].y = 190;
    first_row_[i].Draw(disp);
  }

  titles[0] = "Wh/mi";
  titles[1] = "Solar";
	for (size_t i = 0; i < second_row_.size(); ++i)
	{
		second_row_[i].title = titles[i];
		second_row_[i].x = (i*InfoSquare::DataSqW) + 82;
		second_row_[i].y = 0;
		second_row_[i].Draw(disp);
	}
  DrawSpeed();

}

UI::~UI()
{

}

void UI::UpdateSquare(uint8_t num, etl::string<5>& val)
{
  if(num < 4)
  {
    first_row_[num].UpdateValue(disp, val);
  } else if (num < 6) {
	second_row_[num-4].UpdateValue(disp, val);
  }
}

void UI::DrawSpeed()
{
  // Draw Speed
  disp.SetTextSize(3);
  const char* str2 = "MPH";
  disp.DrawText(136, 105, str2, color_neutral_);
  disp.SetTextSize(4);
  UpdateSpeed(99.9, 0);
}

void UI::UpdateSpeed(float speed, uint8_t regen)
{
	uint16_t speed_color = color_neutral_;
	switch (regen) {
	case 1:
		speed_color = PINK;
		break;
	case 2:
		speed_color = MAGENTA;
		break;
	case 3:
		speed_color = RED;
		break;
	default:
		break;
	}
  // Draw Speed
  disp.SetTextSize(5);
  etl::string<4> s_speed;
  etl::to_string(speed, s_speed, etl::format_spec().precision(1).width(4).fill(0), false);
  disp.DrawText(105, 60, s_speed.c_str(), speed_color);
}

void UI::UpdateBMSTrip(SolarGators::DataModules::OrionBMSRx4* bmsCodes) {
	// BMS Trip Codes
	const char* bms_faults_messages[] = {
		"Internal Cell Communication",
		"Cell Balancing Stuck Off",
		"Weak Cell",
		"Low Cell Voltage",
		"Cell Open Wiring",
		"Current Sensor",
		"Cell Voltage Over 5v",
		"Cell Bank",
		"Weak Pack",
		"Fan Monitor",
		//"Thermistor",
		"Can Communication",
		"Redundant Power Supply",
		"High Voltage Isolation",
		"Invalid Input Supply Voltage",
		"Charge En Relay",
		"Discharge En Relay",
		"Charger Safety Relay",
		"Internal Hardware",
		"Internal Heatsink Thermistor",
		"Internal Logic",
		"Highest Cell Voltage Too High",
		"Lowest Cell Voltage Too Low",
		"Pack Too Hot"
	};

	bool bms_faults_values[24] = {
		bmsCodes->isInternalCellCommunicationFault(),
		bmsCodes->isCellBalancingStuckOffFault(),
		bmsCodes->isWeakCellFault(),
		bmsCodes->isLowCellVoltageFault(),
		bmsCodes->isCellOpenWiringFault(),
		bmsCodes->isCurrentSensorFault(),
		bmsCodes->isCellVoltageOver5vFault(),
		bmsCodes->isCellBankFault(),
		bmsCodes->isWeakPackFault(),
		bmsCodes->isFanMonitorFault(),
		//bmsCodes->isThermistorFault(),
		bmsCodes->isCanCommunicationFault(),
		bmsCodes->isRedundantPowerSupplyFault(),
		bmsCodes->isHighVoltageIsolationFault(),
		bmsCodes->isInvalidInputSupplyVoltageFault(),
		bmsCodes->isChargeenableRelayFault(),
		bmsCodes->isDischargeenableRelayFault(),
		bmsCodes->isChargerSafetyRelayFault(),
		bmsCodes->isInternalHardwareFault(),
		bmsCodes->isInternalHeatsinkThermistorFault(),
		bmsCodes->isInternalLogicFault(),
		bmsCodes->isHighestCellVoltageTooHighFault(),
		bmsCodes->isLowestCellVoltageTooLowFault(),
		bmsCodes->isPackTooHotFault()
	};

	disp.SetTextSize(2);

	//Get the first trip in the list
	etl::string<40> bms_fault = nullptr;
	for (int i = 0; i < 24; i++) {
		if (bms_faults_values[i]) {
			bms_fault = bms_faults_messages[i];
			break;
		}
	}


    if(bms_fault.size() != 0)
    {
        disp.DrawText(0, 133, bms_fault.c_str(), color_fail_);
    }

}

void UI::UpdateMitsubaTrip(SolarGators::DataModules::MitsubaRx2* mitsubaCodes) {
	// Mitsuba Trip Codes
	const char* mitsuba_faults_messages[] = {
	  "AD Sensor Error",
	  "Motor Curr Sensor U Error",
	  "Motor Curr Sensor W Error",
	  "Fet Therm Error",
	  "Batt Volt Sensor Error",
	  "Batt Curr Sensor Error",
	  "Batt Curr Sensor Adj Error",
	  "Motor Curr Sensor Adj Error",
	  "Accel Pos Error",
	  "Cont Volt Sensor Error",
	  "Power System Error",
	  "Over Curr Error",
	  "Over Volt Error",
	  "Over Curr Limit",
	  "Motor System Error",
	  "Motor Lock",
	  "Hall Sensor Short",
	  "Hall Sensor Open"
	};
	bool mitsuba_faults_values[18] = {
			mitsubaCodes->GetAdSensorError(),
			mitsubaCodes->GetMotorSensorUError(),
			mitsubaCodes->GetMotorCurrSensorWError(),
			mitsubaCodes->GetFetThermError(),
			mitsubaCodes->GetBattVoltSensorError(),
			mitsubaCodes->GetBattCurrSensorError(),
			mitsubaCodes->GetBattCurrSensorAdjError(),
			mitsubaCodes->GetMotorCurrSensorAdjError(),
			mitsubaCodes->GetAccelPosError(),
			mitsubaCodes->GetContVoltSensorError(),
			mitsubaCodes->GetPowerSystemError(),
			mitsubaCodes->GetOverCurrError(),
			mitsubaCodes->GetOverVoltError(),
			mitsubaCodes->GetOverCurrLimit(),
			mitsubaCodes->GetMotorSystemError(),
			mitsubaCodes->GetMotorLock(),
			mitsubaCodes->GetHallSensorShort()
	};
	disp.SetTextSize(2);
	//Get the first trip in the list
	const char* mitsuba_fault = nullptr;
	for (int i = 0; i < 18; i++) {
		if (mitsuba_faults_values[i]) {
			mitsuba_fault = mitsuba_faults_messages[i];
			break;
		}
	}

	if(mitsuba_fault != nullptr) {
		// mitsuba_fault
		disp.DrawText(0, 166, mitsuba_fault, color_fail_);
	}
}

void UI::DrawTripCodes()
{
    disp.SetTextSize(2);
    disp.DrawText(0, 120, "BMS Status: ", color_neutral_);
    disp.DrawText(0, 180, "MC Status: ", color_neutral_);
}

void UI::ClearIndicators() {
	disp.SetTextSize(6);
	disp.DrawText(20, 5, " ", GREEN);
	disp.DrawText(267, 5, " ", GREEN);
}

void UI::SetLeftTurn() {
	disp.SetTextSize(6);
	disp.DrawText(20, 5, "<", GREEN);
}

void UI::SetRightTurn() {
	disp.SetTextSize(6);
	disp.DrawText(267, 5, ">", GREEN);
}

void UI::SetHazards() {
	disp.SetTextSize(6);
	disp.DrawText(20, 5, "<", GREEN);
	disp.DrawText(267, 5, ">", GREEN);
}

void UI::SetHeadlights() {
	disp.FillRect(30, 65, 12, 12, GREEN);
	disp.FillRect(50, 65, 12, 12, GREEN);
	disp.FillRect(70, 62, 15, 3, GREEN);
	disp.FillRect(70, 70, 15, 3, GREEN);
	disp.FillRect(70, 77, 15, 3, GREEN);
	disp.FillRect(7, 62, 15, 3, GREEN);
	disp.FillRect(7, 70, 15, 3, GREEN);
	disp.FillRect(7, 77, 15, 3, GREEN);
}

void UI::ClearHeadlights() {
	disp.FillRect(30, 65, 12, 12, BLACK);
	disp.FillRect(50, 65, 12, 12, BLACK);
	disp.FillRect(70, 62, 15, 3, BLACK);
	disp.FillRect(70, 70, 15, 3, BLACK);
	disp.FillRect(70, 77, 15, 3, BLACK);
	disp.FillRect(7, 62, 15, 3, BLACK);
	disp.FillRect(7, 70, 15, 3, BLACK);
	disp.FillRect(7, 77, 15, 3, BLACK);
}

void UI::UpdateSupBat(float voltage) {

	uint32_t batt_color = CYAN;
	if (voltage < 12.1) {
		batt_color = RED;
	}
	disp.FillRect(10, 100, 45, 29, batt_color);
	disp.FillRect(10, 100, 45, 29, batt_color);
	disp.FillRect(18, 95, 10, 5, batt_color);
	disp.FillRect(37, 95, 10, 5, batt_color);
	etl::string<4> s_voltage;
    etl::to_string(voltage, s_voltage, etl::format_spec().precision(1).width(4).fill(0), false);
    disp.SetTextSize(2);
    disp.DrawText(60, 112, s_voltage.c_str(), batt_color);
}

void UI::SetEco() {
	disp.SetTextSize(3);
	disp.DrawText(250, 102, "ECO", OLIVE);
}

void UI::ClearEco() {
	disp.SetTextSize(3);
	disp.DrawText(250, 102, "ECO", BLACK);
}

void UI::SetReverse() {
	disp.SetTextSize(3);
	disp.DrawText(250, 65, "REV", MAGENTA);
}

void UI::ClearReverse() {
	disp.SetTextSize(3);
	disp.DrawText(250, 65, "REV", BLACK);
}

void UI::SetCruise() {
	disp.FillRect(210, 110, 20, 10, ORANGE);
	disp.FillRect(215, 105, 10, 5, ORANGE);
	disp.FillRect(215, 120, 10, 5, ORANGE);
	disp.FillRect(220, 114, 10, 2, BLACK);
	disp.SetTextSize(2);
	disp.DrawText(230, 108, "<", ORANGE);
}

void UI::ClearCruise() {
	disp.FillRect(210, 110, 20, 10, BLACK);
	disp.FillRect(215, 105, 10, 5, BLACK);
	disp.FillRect(215, 120, 10, 5, BLACK);
	disp.SetTextSize(2);
	disp.DrawText(230, 108, "<", BLACK);
}

void UI::SetExternalTrip() {
	disp.SetTextSize(2);
	disp.DrawText(0, 166, "External Kill Switch", color_fail_);
}

} /* namespace Drivers */
} /* namespace SolarGators */
