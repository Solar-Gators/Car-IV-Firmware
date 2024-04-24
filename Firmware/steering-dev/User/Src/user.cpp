/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#include "user.hpp"
#include "threads.hpp"

extern "C" void CPP_UserSetup(void);

ILI9341 display(240, 320);
UI ui(&display); 
Button right_turn_btn = Button(RT_Btn_GPIO_Port, RT_Btn_Pin, 50, GPIO_PIN_SET, false);
Button cruise_plus_btn = Button(Cplus_Btn_GPIO_Port, Cplus_Btn_Pin, 100, GPIO_PIN_SET, false);
Button cruise_minus_btn = Button(Cminus_Btn_GPIO_Port, Cminus_Btn_Pin, 100, GPIO_PIN_SET, false);
Button ptt_btn = Button(PTT_Btn_GPIO_Port, PTT_Btn_Pin, 100, GPIO_PIN_SET, false);

void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");

	

	ui.Init();
	ui.UpdateSpeed(0.00);
	ui.UpdateMode(MODE_ECO);
	ui.UpdateLaps(0);
	ui.UpdateSOC(0.00);
	ui.UpdateBattV(0.00);
	ui.UpdateAuxV(0.00);
	ui.UpdateTemp(0);
	ui.UpdateEfficiency(0);
	ui.UpdateNetPower(0);
	ui.UpdateSolarPower(0);
	ui.UpdateMotorPower(0);

	right_turn_btn.RegisterNormalPressCallback(RightTurnCallback);
	cruise_plus_btn.RegisterNormalPressCallback(CruisePlusCallback);
	cruise_minus_btn.RegisterNormalPressCallback(CruiseMinusCallback);
	ptt_btn.RegisterNormalPressCallback(PTTCallback);

	float speed = 1.5;

	ThreadsStart();

	// while (1) {
	// 	bool state1 = static_cast<bool>(right_turn_btn.ReadPin());
	// 	bool state2 = static_cast<bool>(cruise_plus_btn.ReadPin());
	// 	bool state3 = static_cast<bool>(cruise_minus_btn.ReadPin());
	// 	bool state4 = static_cast<bool>(ptt_btn.ReadPin());
	// 	Logger::LogInfo("States: %u %u %u %u", state1, state2, state3, state4);
	// 	HAL_Delay(100);
	// }


	// while (1) {
	// 	Logger::LogInfo("while\n");
	// 	ui.UpdateSpeed(speed++);
	// 	ui.UpdateMode(MODE_ECO);
	// 	//ui.IncrementTime();
	// 	ui.ToggleLeftTurn();
	// 	ui.ToggleRightTurn();
	// 	ui.UpdatePVStatus(RGB565_RED);
	// 	ui.UpdateBMSStatus(RGB565_RED);
	// 	ui.UpdateMCStatus(RGB565_RED);
	// 	ui.UpdateLaps(232);
	// 	ui.UpdateSOC(101.134);
	// 	ui.UpdateBattV(109.3122);
	// 	ui.UpdateAuxV(10.1234);
	// 	ui.UpdateTemp(23);
	// 	ui.UpdateEfficiency(123);
	// 	ui.UpdateNetPower(1234);
	// 	ui.UpdateSolarPower(1234);
	// 	ui.UpdateMotorPower(1);
	// 	ui.DisplayError1(AUX_HIGH_CURRENT_FAULT);
	// 	ui.DisplayError2(WEAK_CELL_FAULT);
	// 	HAL_Delay(1000);
	// }
}

