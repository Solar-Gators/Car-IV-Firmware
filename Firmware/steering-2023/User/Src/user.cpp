/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#include "user.hpp"
#include "etl/to_string.h"
#include "etl/format_spec.h"
#include "etl/string_utilities.h"
#include "UI.hpp"
#include "fatfs.h"
//#include "FLASH_SECTOR_F4.h"

#define BRIDGESTONE_TIRES 1
#define LENGTH 50
float CHRG_TEMP_LIM = 43.0;
float CHRG_SOC_LIM = 98.0;
uint16_t sd_counter = 0;

using namespace SolarGators;

extern IWDG_HandleTypeDef hiwdg;

extern "C" void CPP_UserSetup(void);
extern "C" void CPP_HandleGPIOInterrupt(uint16_t GPIO_Pin);
extern "C" void Flash_Read_Data(uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);
extern "C" void Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);

void UpdateSignals();
void ReadButtonHold();
void UpdateUI();
void SendCanMsgs();
void HandleEco();
void HandleHeadLights();
void HandleCruise();
void HandleReverse();
void ProcessSD();
void EfficiencyCalc();

osSemaphoreId_t lcdSem = osSemaphoreNew(1, 1, NULL);
osSemaphoreId_t spkrSem = osSemaphoreNew(1, 1, NULL);
//osSemaphoreId_t spkrSem = osSemaphoreNew(1, 1, NULL);
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

FATFS fs;
FIL fil;
FRESULT fresult;
FSIZE_t fil_addr;

uint32_t times[LENGTH];
uint16_t motorRPMs[LENGTH];
uint16_t motorTemps[LENGTH];
uint8_t battAvgTemps[LENGTH];
uint8_t battHighTemps[LENGTH];
float mppt0_voltages[LENGTH];
float mppt1_voltages[LENGTH];
float mppt2_voltages[LENGTH];
float mppt0_currents[LENGTH];
float mppt1_currents[LENGTH];
float mppt2_currents[LENGTH];
float voltages[LENGTH];
float currents[LENGTH];
float SOCs[LENGTH];

bool sdInit = false;
bool indicators_on = false;
float speed = 0;

// OS Configs
osTimerId_t signal_timer_id;
osTimerAttr_t signal_timer_attr =
{
    .name = "lights_led"
};
/* Definitions for UI Updater */
osThreadId_t ui_thread_id;
uint32_t ui_stack[ 256 ];
StaticTask_t ui_control_block;
const osThreadAttr_t ui_thread_attributes = {
  .name = "UI",
  .cb_mem = &ui_control_block,
  .cb_size = sizeof(ui_control_block),
  .stack_mem = &ui_stack[0],
  .stack_size = sizeof(ui_stack),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for SD Logger */
osThreadId_t sd_thread_id;
uint32_t sd_stack[ 256 ];
StaticTask_t sd_control_block;
const osThreadAttr_t sd_thread_attributes = {
  .name = "UI",
  .cb_mem = &sd_control_block,
  .cb_size = sizeof(sd_control_block),
  .stack_mem = &sd_stack[0],
  .stack_size = sizeof(sd_stack),
  .priority = (osPriority_t) osPriorityAboveNormal1,
};

/* Definitions for Button Press Thread */
osThreadId_t btn_thread_id;
uint32_t btn_stack[ 64 ];
StaticTask_t btn_control_block;
const osThreadAttr_t btn_thread_attributes = {
  .name = "UI",
  .cb_mem = &btn_control_block,
  .cb_size = sizeof(btn_control_block),
  .stack_mem = &btn_stack[0],
  .stack_size = sizeof(btn_stack),
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for CAN Tx Thread */
osTimerId_t can_tx_timer_id;
osTimerAttr_t can_tx_timer_attr =
{
    .name = "CAN Tx"
};

/* Definitions for Efficiency Calcs Thread */
osTimerId_t efficiency_timer_id;
osTimerAttr_t efficiency_timer_attr =
{
    .name = "Efficiency Calcs"
};

uint32_t PULSE = 500;

// Wheel Diameter in miles
#if BRIDGESTONE_TIRES
static constexpr float WHEEL_DIAM_MM = 557;
static constexpr float WHEEL_DIAM_FT = WHEEL_DIAM_MM / 304.8;
static constexpr float WHEEL_DIAM_MI = (WHEEL_DIAM_FT / 5280) * 3.14;
#else
static constexpr float WHEEL_DIAM_IN = 23.071;
static constexpr float WHEEL_DIAM_FT = WHEEL_DIAM_IN / 12;
static constexpr float WHEEL_DIAM_MI = (WHEEL_DIAM_IN / 63360) * 3.14;
#endif

SolarGators::DataModules::DataModule* BMS_Rx_0_ptr;
SolarGators::DataModules::DataModule* BMS_Rx_1_ptr;
SolarGators::DataModules::DataModule* BMS_Rx_2_ptr;
SolarGators::DataModules::DataModule* BMS_Rx_4_ptr;
SolarGators::DataModules::DataModule* Motor_Rx_0_ptr;
SolarGators::DataModules::DataModule* Motor_Rx_2_ptr;
SolarGators::DataModules::DataModule* FLights_ptr;
SolarGators::DataModules::DataModule* RLights_ptr;
SolarGators::DataModules::DataModule* PowerBoard_ptr;
SolarGators::DataModules::DataModule* MPPT0_ptr;
SolarGators::DataModules::DataModule* MPPT1_ptr;
SolarGators::DataModules::DataModule* MPPT2_ptr;

void CPP_UserSetup(void)
{
  // Setup Actions
  // Note: These binds really abuse the stack and we should figure out a way to avoid them
  //       since we are heavily constrained.
  // Note: No longer using binds, using etl::delegate should solve our stack problem but still
  //       needs to be tested. This is compile time determined so hoping for better performance also.
  {
    using namespace SolarGators::DataModules;
    // Left Side
    left_turn.action_ = etl::delegate<void(void)>::create<SteeringController, LightsState, &SteeringController::ToggleLeftTurnSignal>();
    cruise_minus.action_ = etl::delegate<void(void)>::create<SteeringController, LightsState, &SteeringController::DecreaseCruiseSpeed>();
    //eco.action_ = etl::delegate<void(void)>::create<HandleEco>();
    headlights.action_ = etl::delegate<void(void)>::create<HandleHeadLights>();
    hazards.action_ = etl::delegate<void(void)>::create<SteeringController, LightsState, &SteeringController::ToggleHazards>();
    // Right Side
    right_turn.action_ = etl::delegate<void(void)>::create<SteeringController, LightsState, &SteeringController::ToggleRightTurnSignal>();
    cruise_plus.action_ = etl::delegate<void(void)>::create<SteeringController, LightsState, &SteeringController::IncreaseCruiseSpeed>();
    //horn.action_ = etl::delegate<void(void)>::create<SteeringController, LightsState, &SteeringController::ToggleHorn>();
    cruise.action_ = etl::delegate<void(void)>::create<HandleCruise>();
    reverse.action_ = etl::delegate<void(void)>::create<HandleReverse>();
  }
  // Add to Button Group
  // Left side
  LightsState.AddButton(&left_turn);
  LightsState.AddButton(&cruise_minus);
  //LightsState.AddButton(&eco);
  LightsState.AddButton(&headlights);
  LightsState.AddButton(&hazards);
  // Right side
  LightsState.AddButton(&right_turn);
  LightsState.AddButton(&cruise_plus);
  //LightsState.AddButton(&horn);
  LightsState.AddButton(&cruise);
  LightsState.AddButton(&reverse);
//   Load the CAN Controller
  CANController.AddRxModule(&BMS_Rx_0);
  BMS_Rx_0_ptr = &BMS_Rx_0;
  CANController.AddRxModule(&BMS_Rx_1);
  BMS_Rx_1_ptr = &BMS_Rx_1;
  CANController.AddRxModule(&BMS_Rx_2);
  BMS_Rx_2_ptr = &BMS_Rx_2;
  CANController.AddRxModule(&BMS_Rx_4);
  BMS_Rx_4_ptr = &BMS_Rx_4;
  CANController.AddRxModule(&Motor_Rx_0);
  Motor_Rx_0_ptr = &Motor_Rx_0;
  CANController.AddRxModule(&Motor_Rx_2);
  Motor_Rx_2_ptr = &Motor_Rx_2;
  CANController.AddRxModule(&PowerBoard);
  PowerBoard_ptr = &PowerBoard;
  CANController.AddRxModule(&FLights);
  FLights_ptr = &FLights;
  CANController.AddRxModule(&RLights);
  RLights_ptr = &RLights;
  CANController.AddRxModule(&MPPT0_Rx_1);
  MPPT0_ptr = &MPPT0_Rx_1;
  CANController.AddRxModule(&MPPT1_Rx_1);
  MPPT1_ptr = &MPPT1_Rx_1;
  CANController.AddRxModule(&MPPT2_Rx_1);
  MPPT2_ptr = &MPPT2_Rx_1;
  CANController.Init();

  // Start Thread that Handles Turn Signal LEDs
  signal_timer_id = osTimerNew((osThreadFunc_t)UpdateSignals, osTimerPeriodic, NULL, &signal_timer_attr);
  if (signal_timer_id == NULL)
  {
      Error_Handler();
  }
  osTimerStart(signal_timer_id, 500);
  // Start Thread that updates screen
  ui_thread_id = osThreadNew((osThreadFunc_t)UpdateUI, NULL, &ui_thread_attributes);
  if (ui_thread_id == NULL)
  {
      Error_Handler();
  }
  // Start Thread for SD logger
    sd_thread_id = osThreadNew((osThreadFunc_t)ProcessSD, NULL, &sd_thread_attributes);
    if (sd_thread_id == NULL)
    {
        Error_Handler();
    }
  // Start Thread that sends CAN Data
  can_tx_timer_id = osTimerNew((osThreadFunc_t)SendCanMsgs, osTimerPeriodic, NULL, &can_tx_timer_attr);
  if (can_tx_timer_id == NULL)
  {
      Error_Handler();
  }
  osTimerStart(can_tx_timer_id, 100);

  // Start Thread that checks for button presses
  btn_thread_id = osThreadNew((osThreadFunc_t)ReadButtonHold, NULL, &btn_thread_attributes);
  if (btn_thread_id == NULL)
  {
	  Error_Handler();
  }

  // Start efficiency calcs thread
  efficiency_timer_id = osTimerNew((osThreadFunc_t)EfficiencyCalc, osTimerPeriodic, NULL, &efficiency_timer_attr);
  if (efficiency_timer_id == NULL)
  {
      Error_Handler();
  }
  osTimerStart(efficiency_timer_id, 1000);

  // For now, ECO will always be on. There is no button for it.
  LightsState.EnableEco();

}

float miles_driven = 0;
float energy_used = 0;
float prev_watts = 0;
float prev_speed = 0;
float current_watts = 0;
uint32_t prev_time = 0;
float efficiency = 0;

void EfficiencyCalc() {

	current_watts = BMS_Rx_0.getPackSumVolt() * BMS_Rx_2.getPackCurrent();

	// calculate energy used in watt-hours
//	float debug1 = ((prev_watts + current_watts) / 2);
//	float debug2 = (float)(xTaskGetTickCount() - prev_time);
//	float debug3 = float(debug2 / (1000 * 3600));
	energy_used += ((prev_watts + current_watts) / 2) *
			(((float)(xTaskGetTickCount() - prev_time)) / (float(1000 * 3600)));

	// calculate the distance traveled since function has last run
	miles_driven += ((prev_speed + speed) / 2) *
			(((float)(xTaskGetTickCount() - prev_time)) / (float(1000 * 3600)));

	prev_time = xTaskGetTickCount();
	prev_watts = current_watts;
	prev_speed = speed;

	if (miles_driven >= 1.0) {

		efficiency = energy_used / miles_driven;
		energy_used = 0;
		miles_driven = 0;

	}

}

void ReadButtonHold() {

	while(1) {

		// Check if horn is pressed
		GPIO_PinState horn = HAL_GPIO_ReadPin(BTN7_GPIO_Port, BTN7_Pin);
		if(horn == GPIO_PIN_SET) {
			LightsState.EnableHorn();
		} else {
			LightsState.DisableHorn();
		}

		// Check if regen is pressed:
		GPIO_PinState regen = HAL_GPIO_ReadPin(BTN8_GPIO_Port, BTN8_Pin);
		if ((regen == GPIO_PIN_SET) && (BMS_Rx_1.getHighTemp() < CHRG_TEMP_LIM) && (BMS_Rx_4.getPackSoc() < CHRG_SOC_LIM)) {
			LightsState.DisableCruise();
			LightsState.EnableRegen();
		} else {
			LightsState.DisableRegen();
		}

		osDelay(15);

	}

}


void UpdateSignals()
{
	if (LightsState.GetLeftTurnStatus() || LightsState.GetRightTurnStatus() || LightsState.GetHazardsStatus()) {

//		osMutexAcquire(spkrSem, osWaitForever);

		if (indicators_on) {
//			__HAL_TIM_SET_PRESCALER(&htim2, 20);
			indicators_on = false;
		} else {
//			__HAL_TIM_SET_PRESCALER(&htim2, 10);
			indicators_on = true;
		}

//		HAL_TIM_Base_Start(&htim2);
//		HAL_TIM_Base_Start_IT(&htim3);
//		osMutexRelease(spkrSem);


	} else {
//		osMutexAcquire(spkrSem, osWaitForever);
//		HAL_TIM_Base_Stop(&htim2);
//		__HAL_TIM_SET_PRESCALER(&htim2, 10);
		indicators_on = false;
//		osMutexRelease(spkrSem);
	}


	// Check if the car has tripped
	if (BMS_Rx_4.isInternalCellCommunicationFault() ||
		BMS_Rx_4.isCellBalancingStuckOffFault() ||
		BMS_Rx_4.isWeakCellFault() ||
		BMS_Rx_4.isLowCellVoltageFault() ||
		BMS_Rx_4.isCellOpenWiringFault() ||
		BMS_Rx_4.isCurrentSensorFault() ||
		BMS_Rx_4.isCellVoltageOver5vFault() ||
		BMS_Rx_4.isCellBankFault() ||
		BMS_Rx_4.isWeakPackFault() ||
		BMS_Rx_4.isFanMonitorFault() ||
		//BMS_Rx_4.isThermistorFault() ||
		BMS_Rx_4.isCanCommunicationFault() ||
		BMS_Rx_4.isRedundantPowerSupplyFault() ||
		BMS_Rx_4.isHighVoltageIsolationFault() ||
		BMS_Rx_4.isInvalidInputSupplyVoltageFault() ||
		BMS_Rx_4.isChargeenableRelayFault() ||
		BMS_Rx_4.isDischargeenableRelayFault() ||
		BMS_Rx_4.isChargerSafetyRelayFault() ||
		BMS_Rx_4.isInternalHardwareFault() ||
		BMS_Rx_4.isInternalHeatsinkThermistorFault() ||
		BMS_Rx_4.isInternalLogicFault() ||
		BMS_Rx_4.isHighestCellVoltageTooHighFault() ||
		BMS_Rx_4.isLowestCellVoltageTooLowFault() ||
		BMS_Rx_4.isPackTooHotFault() ||
		(!RLights.getContactorStatus())) {

		HAL_GPIO_TogglePin(TRIP_GPIO_Port, TRIP_Pin);

	}

	if (FLights.GetBreaksVal()) {
		LightsState.DisableCruise();
	}

}

SolarGators::Drivers::ILI9341 Display(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);

void UpdateUI()
{
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // LCD
  osSemaphoreAcquire(lcdSem, osWaitForever);
  Display.Init();

  // This will initialize the UI
  SolarGators::Drivers::UI ui(Display);
  // Indicate ECO. It is permanently enabled for now.
  ui.SetEco();
  osSemaphoreRelease(lcdSem);

  // Set Format
  static constexpr etl::format_spec format(10,5,1,false,false,false,false,'0');
  while(1)
  {
    etl::string<5> buff;
    // Update Pack Temperature (C)
    etl::to_string(BMS_Rx_1.getHighTemp(), buff, format, false);
	osSemaphoreAcquire(lcdSem, osWaitForever);
	ui.UpdateSquare(0, buff);
	osSemaphoreRelease(lcdSem);

    // Update SOC
    etl::to_string(BMS_Rx_4.getPackSoc(), buff, format, false);
    osSemaphoreAcquire(lcdSem, osWaitForever);
    ui.UpdateSquare(1, buff);
    osSemaphoreRelease(lcdSem);

    // Update Output Power
    etl::to_string((BMS_Rx_0.getPackSumVolt() * BMS_Rx_2.getPackCurrent()), buff, format, false);
    osSemaphoreAcquire(lcdSem, osWaitForever);
    ui.UpdateSquare(2, buff);
    osSemaphoreRelease(lcdSem);

    // Update Current
    etl::to_string(BMS_Rx_2.getPackCurrent(), buff, format, false);
    osSemaphoreAcquire(lcdSem, osWaitForever);
    ui.UpdateSquare(3, buff);
    osSemaphoreRelease(lcdSem);

    // Update Efficiency
    if (efficiency) {
    	etl::to_string(efficiency, buff, format, false);
		osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.UpdateSquare(4, buff);
		osSemaphoreRelease(lcdSem);
    }

    // Update Solar Power
    float array_power =
    		  (MPPT0_Rx_1.getOutputVoltage() * MPPT0_Rx_1.getOutputCurrent())
			+ (MPPT1_Rx_1.getOutputVoltage() * MPPT1_Rx_1.getOutputCurrent())
			+ (MPPT2_Rx_1.getOutputVoltage() * MPPT2_Rx_1.getOutputCurrent());
    etl::to_string(array_power, buff, format, false);
	osSemaphoreAcquire(lcdSem, osWaitForever);
	ui.UpdateSquare(5, buff);
	osSemaphoreRelease(lcdSem);

    // Update Speed
    speed = Motor_Rx_0.GetMotorRPM() * WHEEL_DIAM_MI * 60;
    osSemaphoreAcquire(lcdSem, osWaitForever);
    ui.UpdateSpeed(speed, LightsState.GetRegen());
    osSemaphoreRelease(lcdSem);
    //draw trip codes
    osSemaphoreAcquire(lcdSem, osWaitForever);
    ui.UpdateMitsubaTrip(&Motor_Rx_2);
    ui.UpdateBMSTrip(&BMS_Rx_4);
    osSemaphoreRelease(lcdSem);

    // Update Turn Indicators
    if (indicators_on) {
    	osSemaphoreAcquire(lcdSem, osWaitForever);
    	if (LightsState.GetHazardsStatus()) {
    		ui.SetHazards();
    	} else if (LightsState.GetLeftTurnStatus()) {
    		ui.SetLeftTurn();
    	} else if (LightsState.GetRightTurnStatus()) {
    		ui.SetRightTurn();
    	}
    	osSemaphoreRelease(lcdSem);
    } else {
    	osSemaphoreAcquire(lcdSem, osWaitForever);
    	ui.ClearIndicators();
    	osSemaphoreRelease(lcdSem);
    }

    // Update Headlight Indicator
    if (LightsState.GetHeadlightsStatus()) {
    	osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.SetHeadlights();
		osSemaphoreRelease(lcdSem);
    } else {
    	osSemaphoreAcquire(lcdSem, osWaitForever);
    	ui.ClearHeadlights();
    	osSemaphoreRelease(lcdSem);
    }

    // Update 12V Batt Monitor
    osSemaphoreAcquire(lcdSem, osWaitForever);
	ui.UpdateSupBat(PowerBoard.GetSupBatVoltage());
	osSemaphoreRelease(lcdSem);

	// Update Reverse
	if (LightsState.GetReverseStatus()) {
		osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.SetReverse();
		osSemaphoreRelease(lcdSem);
	} else {
		osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.ClearReverse();
		osSemaphoreRelease(lcdSem);
	}

	// Update Cruise
	if (LightsState.GetCruiseEnabledStatus()) {
		osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.SetCruise();
		osSemaphoreRelease(lcdSem);
	} else {
		osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.ClearCruise();
		osSemaphoreRelease(lcdSem);
	}

	// Display external kill switch error
	if (!RLights.getContactorStatus()) {
		osSemaphoreAcquire(lcdSem, osWaitForever);
		ui.SetExternalTrip();
		osSemaphoreRelease(lcdSem);
	}

    osDelay(40); // Aim for 20hz
  }
}

void SendCanMsgs()
{
  //osSemaphoreAcquire(canSem, osWaitForever);
  // Send the lights state
  HAL_IWDG_Refresh(&hiwdg);
  CANController.Send(&LightsState);
  // Request motor controller data
  McReq.SetRequestAllFrames();
  CANController.Send(&McReq);
//  osDelay(2);
//  osSemaphoreRelease(canSem);
}

void CPP_HandleGPIOInterrupt(uint16_t GPIO_Pin)
{
  LightsState.HandlePress(GPIO_Pin);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANController.SetRxFlag();
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HandleEco()
{
  LightsState.ToggleEco();
}
void HandleHeadLights()
{
  LightsState.ToggleHeadlights();
}
void HandleCruise()
{
	// Do not enable cruise if mechanical or regen brakes are active
	if ((!FLights.GetBreaksVal()) && (LightsState.GetRegen() == 0)) {
		uint8_t debug1 = static_cast<uint8_t>(FLights.GetThrottleVal() >> 5);
		LightsState.SetCruiseSpeed(debug1);
		LightsState.ToggleCruise();
	}
}
void HandleReverse()
{
	if (speed <= 2.0) {
		LightsState.ToggleReverse();
	}
}

void ProcessSD(void) {

	char nameArray[10] = {'l', 'o', 'g', '0', '0', '.', 't', 'x', 't'};
	uint32_t nums[3];
	char chars[2];

	// Read from flash memory
	Flash_Read_Data(0x08070000 , nums, 2);
	chars[0] = (char)nums[0];
	chars[1] = (char)nums[1];

	// Update nameArray
	nameArray[3] = nums[1];
	nameArray[4] = nums[0];

	switch (chars[0]) {
	case '0':
		chars[0] = '1';
		break;
	case '1':
		chars[0] = '2';
		break;
	case '2':
		chars[0] = '3';
		break;
	case '3':
		chars[0] = '4';
		break;
	case '4':
		chars[0] = '5';
		break;
	case '5':
		chars[0] = '6';
		break;
	case '6':
		chars[0] = '7';
		break;
	case '7':
		chars[0] = '8';
		break;
	case '8':
		chars[0] = '9';
		break;
	case '9':
		chars[0] = '0';
		break;
	default:
		// Flash memory is garbage, reinitialize
		chars[0] = '1';
		chars[1] = '0';
		break;
	}

	if (chars[0] == '0') {
		switch (chars[1]) {
		case '0':
			chars[1] = '1';
			break;
		case '1':
			chars[1] = '2';
			break;
		case '2':
			chars[1] = '3';
			break;
		case '3':
			chars[1] = '4';
			break;
		case '4':
			chars[1] = '5';
			break;
		case '5':
			chars[1] = '6';
			break;
		case '6':
			chars[1] = '7';
			break;
		case '7':
			chars[1] = '8';
			break;
		case '8':
			chars[1] = '9';
			break;
		case '9':
			chars[1] = '0';
			break;
		default:
			chars[1] = '0';
			break;
		}
	}

	// Load next value into flash memory
	nums[0] = (uint32_t)chars[0];
	nums[1] = (uint32_t)chars[1];
	Flash_Write_Data(0x08070000 , (uint32_t *)nums, 2);

//	char buffer[5];
//	char num0 = '0';
//	char num1 = '0';
//
//	if (!sdInit && HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin)) {
//
//		osSemaphoreAcquire(lcdSem, osWaitForever);
//		Display.Reset();
//
//		fresult = f_mount(&fs, "", 0);
//		fresult = f_open(&fil, "lognum.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
//		f_gets(buffer, sizeof(buffer), &fil);
//		fresult = f_close(&fil);
//
//		if (buffer[1]) {
//			nameArray[4] = buffer[1];
//		}
//
//		if (buffer[0]) {
//			nameArray[3] = buffer[0];
//		}
//
//		//osDelay(5000);
//		uint32_t count = 0;
//		while (count < 100000) {
//			count++;
//		}
//
//		if (buffer[0] && buffer[1]) {
//
//			switch (buffer[1]) {
//			case '0':
//				num1 = '1';
//				break;
//			case '1':
//				num1 = '2';
//				break;
//			case '2':
//				num1 = '3';
//				break;
//			case '3':
//				num1 = '4';
//				break;
//			case '4':
//				num1 = '5';
//				break;
//			case '5':
//				num1 = '6';
//				break;
//			case '6':
//				num1 = '7';
//				break;
//			case '7':
//				num1 = '8';
//				break;
//			case '8':
//				num1 = '9';
//				break;
//			case '9':
//				num1 = '0';
//				break;
//			default:
//				break;
//			}
//
//			if (num1 == '0') {
//				switch (buffer[0]) {
//				case '0':
//					num0 = '1';
//					break;
//				case '1':
//					num0 = '2';
//					break;
//				case '2':
//					num0 = '3';
//					break;
//				case '3':
//					num0 = '4';
//					break;
//				case '4':
//					num0 = '5';
//					break;
//				case '5':
//					num0 = '6';
//					break;
//				case '6':
//					num0 = '7';
//					break;
//				case '7':
//					num0 = '8';
//					break;
//				case '8':
//					num0 = '9';
//					break;
//				case '9':
//					num0 = '0';
//					break;
//				default:
//					break;
//				}
//			}
//
//			fresult = f_open(&fil, "lognum.txt", FA_OPEN_ALWAYS | FA_WRITE);
//			fresult = (FRESULT)f_putc(num0, &fil);
//			fresult = (FRESULT)f_putc(num1, &fil);
//			fresult = f_close(&fil);
//			fresult = f_mount(NULL, "", 0);
//		} else {
//			fresult = f_open(&fil, "lognum.txt", FA_OPEN_ALWAYS | FA_WRITE);
//			fresult = (FRESULT)f_putc('0', &fil);
//			fresult = (FRESULT)f_putc('1', &fil);
//			fresult = f_close(&fil);
//			fresult = f_mount(NULL, "", 0);
//		}
	sdInit = true;
	//}
//		Display.Resume();
//		osSemaphoreRelease(lcdSem);
//	}

	while(1) {


	//char *ptr = &nameArray[0];

	if (sd_counter < LENGTH) {

		times[sd_counter] = xTaskGetTickCount();
		motorRPMs[sd_counter] = Motor_Rx_0.GetMotorRPM();
		motorTemps[sd_counter] = Motor_Rx_0.GetFetTemp();
		currents[sd_counter] = BMS_Rx_2.getPackCurrent();
		battAvgTemps[sd_counter] = BMS_Rx_1.getAvgTemp();
		battHighTemps[sd_counter] = BMS_Rx_1.getHighTemp();
		voltages[sd_counter] = BMS_Rx_0.getPackSumVolt();
		SOCs[sd_counter] = BMS_Rx_4.getPackSoc();
		mppt0_voltages[sd_counter] = MPPT0_Rx_1.getOutputVoltage();
		mppt1_voltages[sd_counter] = MPPT1_Rx_1.getOutputVoltage();
		mppt2_voltages[sd_counter] = MPPT2_Rx_1.getOutputVoltage();
		mppt0_currents[sd_counter] = MPPT0_Rx_1.getOutputCurrent();
		mppt1_currents[sd_counter] = MPPT1_Rx_1.getOutputCurrent();
		mppt2_currents[sd_counter] = MPPT2_Rx_1.getOutputCurrent();
		sd_counter++;

	} else {

		sd_counter = 0;
		// HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) &&
		if (sdInit) {

			osSemaphoreAcquire(lcdSem, osWaitForever);
			Display.Reset();

			fresult = f_mount(&fs, "", 0);

			if (fresult == FR_OK) {

				fresult = f_open(&fil, nameArray, FA_OPEN_ALWAYS | FA_WRITE);

				fresult = f_lseek(&fil, fil.fptr);

				if (fil_addr) {
					fresult = f_lseek(&fil, fil_addr);
				} else {
					fresult = (FRESULT)f_puts("Time (ms),State of Charge,Battery Voltage,Battery Current,Battery Avg Temp (C),Battery High Temp (C),Motor RPM,Motor Temp,MPPT0 Voltage,MPPT0 Current,MPPT1 Voltage,MPPT1 Current,MPPT2 Voltage,MPPT2 Current\n", &fil);
				}

				static constexpr etl::format_spec format_long(10,8,1,false,false,false,false,'0');
				static constexpr etl::format_spec format(10,5,1,false,false,false,false,'0');
				etl::string<5> buff;
				etl::string<8> buff_long;

				for (int i = 0; i < LENGTH; i++) {

					etl::to_string(times[i], buff_long, format_long, false);
					fresult = (FRESULT)f_puts(buff_long.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(SOCs[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(voltages[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(currents[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(battAvgTemps[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(battHighTemps[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(motorRPMs[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(motorTemps[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);

					etl::to_string(mppt0_voltages[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(mppt0_currents[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);

					etl::to_string(mppt1_voltages[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(mppt1_currents[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);

					etl::to_string(mppt2_voltages[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_putc(',', &fil);
					etl::to_string(mppt2_currents[i], buff, format, false);
					fresult = (FRESULT)f_puts(buff.c_str(), &fil);
					fresult = (FRESULT)f_puts("\n", &fil);

				}

//				etl::to_string(BMS_Rx_4.getPackSoc(), buff, format, false);
//
//				float myFloat = 15.23;
//				char soc = (char)myFloat;
//
//				fresult = (FRESULT)f_puts(buff.c_str(), &fil);
//				fresult = (FRESULT)f_puts("\n", &fil);
//				myFloat /= 100;
//				soc = (char)myFloat;
//				fresult = (FRESULT)f_putc(soc, &fil);
//				fresult = (FRESULT)f_puts("\n", &fil);
//
//				fresult = (FRESULT)f_putc(nameArray[2], &fil);
//				fresult = (FRESULT)f_puts("\n", &fil);
				//fresult = (FRESULT)f_puts("hello\n\n", &fil);

				fil_addr = fil.fptr;

				fresult = f_close(&fil);

				fresult = f_mount(NULL, "", 0);

				Display.Resume();
				osSemaphoreRelease(lcdSem);


		}

		times[sd_counter] = xTaskGetTickCount();
		motorRPMs[sd_counter] = Motor_Rx_0.GetMotorRPM();
		motorTemps[sd_counter] = Motor_Rx_0.GetFetTemp();
		currents[sd_counter] = BMS_Rx_2.getPackCurrent();
		voltages[sd_counter] = BMS_Rx_0.getPackSumVolt();
		SOCs[sd_counter] = BMS_Rx_4.getPackSoc();
		battAvgTemps[sd_counter] = BMS_Rx_1.getAvgTemp();
		battHighTemps[sd_counter] = BMS_Rx_1.getHighTemp();
		mppt0_voltages[sd_counter] = MPPT0_Rx_1.getOutputVoltage();
		mppt1_voltages[sd_counter] = MPPT1_Rx_1.getOutputVoltage();
		mppt2_voltages[sd_counter] = MPPT2_Rx_1.getOutputVoltage();
		mppt0_currents[sd_counter] = MPPT0_Rx_1.getOutputCurrent();
		mppt1_currents[sd_counter] = MPPT1_Rx_1.getOutputCurrent();
		mppt2_currents[sd_counter] = MPPT2_Rx_1.getOutputCurrent();
		sd_counter++;

	}



	}

	osDelay(100);

	}
}
