#include "user.hpp"
#include "threads.hpp"

extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;

/* Initialize CAN frames and devices */
CANDevice candev1 = CANDevice(&hcan1);

ILI9341 display(240, 320);
UI ui(&display); 
Button left_turn_btn = Button(LT_Btn_GPIO_Port, LT_Btn_Pin, 75, GPIO_PIN_SET, false);
Button mode_btn = Button(Mode_Btn_GPIO_Port, Mode_Btn_Pin, 75, GPIO_PIN_SET, false);
Button regen_btn = Button(RGN_Btn_GPIO_Port, RGN_Btn_Pin, 75, GPIO_PIN_SET, false);
Button horn_btn = Button(Horn_Btn_GPIO_Port, Horn_Btn_Pin, 75, GPIO_PIN_SET, false);
Button mc_btn = Button(MC_Btn_GPIO_Port, MC_Btn_Pin, 75, GPIO_PIN_SET, false);
Button right_turn_btn = Button(RT_Btn_GPIO_Port, RT_Btn_Pin, 50, GPIO_PIN_SET, false);
Button cruise_plus_btn = Button(Cplus_Btn_GPIO_Port, Cplus_Btn_Pin, 75, GPIO_PIN_SET, false);
Button cruise_minus_btn = Button(Cminus_Btn_GPIO_Port, Cminus_Btn_Pin, 75, GPIO_PIN_SET, false);
Button ptt_btn = Button(PTT_Btn_GPIO_Port, PTT_Btn_Pin, 75, GPIO_PIN_SET, false);
Button pv_btn = Button(PV_Btn_GPIO_Port, PV_Btn_Pin, 75, GPIO_PIN_SET, false);

HAL_StatusTypeDef CAN_Modules_Init() {
	HAL_StatusTypeDef status = HAL_OK;

	status = CANController::AddDevice(&candev1);
	if (status != HAL_OK) {
		Logger::LogError("Failed to add CAN device\n");
		return status;
	}
	CANController::AddRxMessage(&MitsubaFrame0::Instance());
	CANController::AddRxMessage(&MitsubaFrame1::Instance());
	CANController::AddRxMessage(&MitsubaFrame2::Instance());
	CANController::AddRxMessage(&MPPTInputMeasurementsFrame1::Instance());
	CANController::AddRxMessage(&MPPTInputMeasurementsFrame2::Instance());
	CANController::AddRxMessage(&MPPTInputMeasurementsFrame3::Instance());
	CANController::AddRxMessage(&MPPTOutputMeasurementsFrame1::Instance());
	CANController::AddRxMessage(&MPPTOutputMeasurementsFrame2::Instance());
	CANController::AddRxMessage(&MPPTOutputMeasurementsFrame3::Instance());
	CANController::AddRxMessage(&BMSFrame0::Instance());
	CANController::AddRxMessage(&BMSFrame1::Instance());
	CANController::AddRxMessage(&BMSFrame2::Instance());
	CANController::AddRxMessage(&BMSFrame3::Instance());
	CANController::AddFilterAll();

	status = CANController::Start();
	if (status != HAL_OK) {
		Logger::LogError("Failed to start CAN controller\n");
		return status;
	}

	return status;
}

HAL_StatusTypeDef Buttons_Init() {
	left_turn_btn.RegisterNormalPressCallback(LeftTurnCallback);
	mode_btn.RegisterNormalPressCallback(ModeCallback);
	mode_btn.RegisterLongPressCallback(ModeLongCallback, 800, false);
	regen_btn.RegisterNormalPressCallback(RegenCallback);
	horn_btn.RegisterNormalPressCallback(HornCallback);
	mc_btn.RegisterNormalPressCallback(MCCallback);

	// TODO: Get rid of this once PV button is working
	mc_btn.RegisterLongPressCallback(PVCallback, 800, false);
	
	right_turn_btn.RegisterNormalPressCallback(RightTurnCallback);
	cruise_plus_btn.RegisterNormalPressCallback(CruisePlusCallback);
	cruise_minus_btn.RegisterNormalPressCallback(CruiseMinusCallback);
	ptt_btn.RegisterNormalPressCallback(PTTCallback);
	pv_btn.RegisterNormalPressCallback(PVCallback);

	return HAL_OK;
}

void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");

	if (CAN_Modules_Init() != HAL_OK) {
		Logger::LogError("CAN modules failed to initialize\n");
		Error_Handler();
	}

	if (Buttons_Init() != HAL_OK) {
		Logger::LogError("Buttons failed to initialize\n");
		Error_Handler();
	}

	ui.Init();
	// ui.UpdateSpeed(0.00);
	ui.UpdateMode(MODE_ECO);
	// ui.UpdateLaps(0);
	// ui.UpdateSOC(0.00);
	// ui.UpdateBattV(0.00);
	ui.UpdateAuxV(0.00);
	// ui.UpdateTemp(0);
	ui.UpdateEfficiency(0);
	// ui.UpdateNetPower(0);
	ui.UpdateSolarPower(0);
	ui.UpdateMotorPower(0);
	ui.UpdateMCStatus(RGB565_RED);
	ui.UpdatePVStatus(RGB565_RED);
	ui.UpdateBMSStatus(RGB565_RED);

	ThreadsStart();

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

// Modes: 0 = Eco, 1 = Power, 2 = Reverse
void SendMode(SteeringModeTypeDef mode) {
	osMutexAcquire(ui_mutex, osWaitForever);

	switch (mode) {
		case MODE_ECO:
			DriverControlsFrame1::Instance().SetDriveMode(MotorMode::ECO);
			DriverControlsFrame1::Instance().SetDriveDirection(MotorDirection::FORWARD);
			ui.UpdateMode(MODE_ECO);
			break;
		case MODE_PWR:
			DriverControlsFrame1::Instance().SetDriveMode(MotorMode::POWER);
			DriverControlsFrame1::Instance().SetDriveDirection(MotorDirection::FORWARD);
			ui.UpdateMode(MODE_PWR);
			break;
		case MODE_REV:
			DriverControlsFrame1::Instance().SetDriveMode(MotorMode::ECO);
			DriverControlsFrame1::Instance().SetDriveDirection(MotorDirection::REVERSE);
			ui.UpdateMode(MODE_REV);
			break;
		default:
			Logger::LogError("Invalid mode\n");
			return;
	}

	osMutexRelease(ui_mutex);

	CANController::Send(&DriverControlsFrame1::Instance());
}