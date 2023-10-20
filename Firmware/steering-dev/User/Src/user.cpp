/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#include "user.hpp"

extern "C" void CPP_UserSetup(void);

/* Thread function prototypes */
/**
 * @brief Periodic task that runs at 1Hz
 * Info and power data is updated on the display
*/
void PeriodicTask2(void *argument);
/**
 * @brief Periodic task that runs at 5Hz
*/
void PeriodicTask10(void *argument);

/*  */

/* Periodic timer definitions */
osTimerAttr_t periodic2_timer_attr = {
	.name = "Periodic Task 2",
	.attr_bits = 0,
	.cb_mem = NULL,
	.cb_size = 0,
};
osTimerAttr_t periodic10_timer_attr = {
	.name = "Periodic Task 10",
	.attr_bits = 0,
	.cb_mem = NULL,
	.cb_size = 0,
};



void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");


	ILI9341 display(240, 320);

	UI ui(&display);

	ui.Init();
	ui.UpdateSpeed(35.23);
	ui.UpdateMode(MODE_PWR);

	float speed = 1.5;

	while (1) {
		Logger::LogInfo("while\n");
		ui.UpdateSpeed(speed++);
		ui.UpdateMode(MODE_ECO);
		//ui.IncrementTime();
		ui.ToggleLeftTurn();
		ui.ToggleRightTurn();
		ui.UpdatePVStatus(RGB565_RED);
		ui.UpdateBMSStatus(RGB565_RED);
		ui.UpdateMCStatus(RGB565_RED);
		ui.UpdateLaps(232);
		ui.UpdateSOC(101.134);
		ui.UpdateBattV(109.3122);
		ui.UpdateAuxV(10.1234);
		ui.UpdateTemp(23);
		ui.UpdateEfficiency(123);
		ui.UpdateNetPower(1234);
		ui.UpdateSolarPower(1234);
		ui.UpdateMotorPower(1);
		ui.DisplayError1(AUX_HIGH_CURRENT_FAULT);
		ui.DisplayError2(WEAK_CELL_FAULT);
		HAL_Delay(1000);
	}
}