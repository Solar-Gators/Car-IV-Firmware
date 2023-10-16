/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#include "user.hpp"

void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");


	ILI9341 display(240, 320);
	display.Init();
	display.ClearScreen(0x0000);
	display.SetRotation(1);

	display.DrawRect(0, 0, 220, 60, RGB565_WHITE);
	display.DrawRect(220, 0, 100, 60, RGB565_WHITE);
	display.DrawRect(0, 60, 320, 30, RGB565_WHITE);
	display.DrawRect(0, 90, 320, 150, RGB565_WHITE);


	display.SetTextSize(4);
	display.DrawText(16, 15, "20.8 MPH", RGB565_YELLOW);

	display.SetTextSize(3);
	display.DrawText(242, 20, "ECO", RGB565_GREEN);

	display.SetTextSize(2);
	display.DrawText(20, 67, "<    PV   BMS   MC    >", RGB565_GREEN);

	display.SetTextSize(4);
	display.DrawText(10, 103, "00:00", RGB565_CYAN);

	display.SetTextSize(2);
	display.DrawText(10, 142, "Laps: 0", RGB565_WHITE);
	display.DrawText(10, 161, "SOC:  100", RGB565_WHITE);
	display.DrawText(10, 180, "Batt: 109.2", RGB565_WHITE);
	display.DrawText(10, 199, "Aux:  12.4", RGB565_WHITE);
	display.DrawText(10, 218, "Temp: 40", RGB565_WHITE);

	display.SetTextSize(1);
	display.DrawText(160, 100, "12V High Current", RGB565_RED);
	display.DrawText(160, 112, "Weak Cell Fault", RGB565_RED);

	display.SetTextSize(3);
	display.DrawText(160, 128, "  14", RGB565_YELLOW);
	display.DrawText(160, 156, " 545", RGB565_CYAN);
	display.DrawText(160, 184, " 693", RGB565_PINK);
	display.DrawText(160, 212, "1238", RGB565_DARK_GREEN);
	
	display.SetTextSize(2);
	display.DrawText(232, 135, "Wh/mi", RGB565_YELLOW);
	display.DrawText(232, 163, "W Net", RGB565_CYAN);
	display.DrawText(232, 191, "W Solar", RGB565_PINK);
	display.DrawText(232, 219, "W Motor", RGB565_DARK_GREEN);

	
	while (1) {
		Logger::LogInfo("while\n");
		HAL_Delay(1000);
	}
}