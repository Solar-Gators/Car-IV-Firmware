/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#include "user.hpp"
#include "sg_can.hpp"
#include "CustomBMSFrames.hpp"
#include "MitsubaFrames.hpp"
#include "MPPTFrames.hpp"
#include "DriverControls.hpp"

#define UI_THREAD_STACK_SIZE  256

extern "C" void CPP_UserSetup(void);
extern "C" CAN_HandleTypeDef hcan1;

void CAN_task(void *argument);
void UTime(void *argument);

ILI9341 display(240, 320);
UI ui(&display);
uint32_t previous_time = 0;

// Can Fields
CANDevice CAN = CANDevice(&hcan1);
CANFrame power_board = CANFrame(0x800, CAN_ID_EXT, CAN_RTR_DATA, 8, UPowerBoard);

// Fields related to efficiency and speed calculation
static constexpr float WHEEL_DIAM_MI = (0.0010867658F); // wheel diameter in miles / rotation
float prevspeed = 0.001;
float prevwatts = 0;
float current = 0;
float volt = 0;
float speed = 1.5;
float energy_used = 0;
float miles_driven = 0;
uint8_t updateEfficiency = 0;
float efficiency = 0;
float solarpower = 0.0f;

// UI fields and thread definitions
osThreadId_t ui_threadHandle;
uint32_t ui_thread_task_buffer[UI_THREAD_STACK_SIZE];
StaticTask_t ui_threadHandle_tcb_;
osThreadAttr_t ui_thread_attr = {
	.name = "ui_thread_task",
	.attr_bits = osThreadDetached,
	.cb_mem = &ui_threadHandle_tcb_,
	.cb_size = sizeof(ui_threadHandle_tcb_),
	.stack_mem = &ui_thread_task_buffer[0],
	.stack_size = sizeof(ui_thread_task_buffer),
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0,
};
osMessageQueueId_t ui_queueHandle;
// FIFO for UI update messages:
const osMessageQueueAttr_t ui_queue_attr = {
	.name="UI Queue"
};

//Task for testing:
osThreadId_t can_threadhandle;
uint32_t can_thread_task_buffer[UI_THREAD_STACK_SIZE];
StaticTask_t can_threadHandle_tcb_;
osThreadAttr_t can_thread_attr = {
	.name = "can_thread_task",
	.attr_bits = osThreadDetached,
	.cb_mem = &can_threadHandle_tcb_,
	.cb_size = sizeof(can_threadHandle_tcb_),
	.stack_mem = &can_thread_task_buffer[0],
	.stack_size = sizeof(can_thread_task_buffer),
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0,
};

// Timer:
osTimerId_t carTimer;
const osTimerAttr_t carTimer_attr = {
	.name = "Car Timer",
};

// Button BTN0 = Button(BTN0_GPIO_Port, BTN0_Pin, 50, GPIO_PIN_SET, false);
// Button BTN1 = Button(BTN1_GPIO_Port, BTN1_Pin, 50, GPIO_PIN_SET, false);
// Button BTN8 = Button(BTN8_GPIO_Port, BTN8_Pin, 50, GPIO_PIN_SET, false);
// Button BTN2 = Button(BTN2_GPIO_Port, BTN2_Pin, 50, GPIO_PIN_SET, false);
// Button BTN3 = Button(BTN3_GPIO_Port, BTN3_Pin, 50, GPIO_PIN_SET, false);
//  Button BTN4 = Button(BTN4_GPIO_Port, BTN4_Pin, 50, GPIO_PIN_SET, false);
//  Button BTN5 = Button(BTN5_GPIO_Port, BTN5_Pin, 50, GPIO_PIN_SET, false);
// Button BTN9 = Button(BTN9_GPIO_Port, BTN9_Pin, 50, GPIO_PIN_SET, false);
//Button BTN6 = Button(BTN6_GPIO_Port, BTN6_Pin, 50, GPIO_PIN_SET, false);


void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");
	//ui.Init();
	osKernelInitialize();
	//ui_queueHandle = osMessageQueueNew(8, sizeof(UIQueue_Msg), &ui_queue_attr);
	//ui_threadHandle = osThreadNew(UI_task, NULL, &ui_thread_attr);
	//can_threadhandle = osThreadNew(CAN_task, NULL, &can_thread_attr);
	//CAN_Init();
	//carTimer = osTimerNew(UTime, osTimerPeriodic, NULL, &carTimer_attr);
	//osTimerStart(carTimer, 1000);
	//Button_Init();
}

void UTime(void *argument){
	UIQueue_Msg *message = new UIQueue_Msg(Timer, NULL);
	osMessageQueuePut(ui_queueHandle, &message, 0, osWaitForever);
}
void CAN_task(void *argument){
	while(true){
		BMSFrame0::Instance().SetPackVoltage(12);
		CANController::Send(&BMSFrame0::Instance());
		osDelay(1000);

		BMSFrame1::Instance().SetPackCurrent(10);
		CANController::Send(&BMSFrame1::Instance());
		osDelay(1000);

		uint8_t dt[8] = {
			1,0,0b11010000,0b00100100, 0b11101000, 0b00101111,0,0
		};
		MitsubaFrame0::Instance().LoadData(dt, 8);
		CANController::Send(&MitsubaFrame0::Instance());
		osDelay(1000);

		BMSFrame3::Instance().SetHighChargeCurrentFault(true);
		BMSFrame3::Instance().SetPackSoC(36);
		CANController::Send(&BMSFrame3::Instance());
		osDelay(1000);

		BMSFrame3::Instance().SetHighChargeCurrentFault(false);
		CANController::Send(&BMSFrame3::Instance());
		osDelay(1000);

		BMSFrame2::Instance().SetHighTemp(60);
		CANController::Send(&BMSFrame2::Instance());
		osDelay(1000);

		
		dt[0] = 0b00000000;
		MitsubaFrame1::Instance().LoadData(dt, 5);
		CANController::Send(&MitsubaFrame1::Instance());
		osDelay(1000);

		dt[0] = 0b10000000;
		MitsubaFrame1::Instance().LoadData(dt, 5);
		CANController::Send(&MitsubaFrame1::Instance());
		osDelay(1000);

		dt[0] = 0b11111111;
		dt[1] = 0b11111111;
		dt[2] = 0b11111111;
		dt[3] = 0b11111111;
		dt[4] = 0b11111111;
		dt[5] = 0b11111111;
		dt[6] = 0b11111111;
		dt[7] = 0b11111111;
		MPPTOutputMeasurementsFrame1::Instance().LoadData(dt, 8);
		MPPTOutputMeasurementsFrame2::Instance().LoadData(dt, 8);
		MPPTOutputMeasurementsFrame3::Instance().LoadData(dt, 8);
		CANController::Send(&MPPTOutputMeasurementsFrame1::Instance());
		CANController::Send(&MPPTOutputMeasurementsFrame2::Instance());
		CANController::Send(&MPPTOutputMeasurementsFrame3::Instance());
		osDelay(1000);

		dt[1] = 0b11001000;
		dt[0] = 0b00000001;
		power_board.LoadData(dt, 7);
		CANController::Send( &power_board);
		osDelay(1000);
		
		// HAL_GPIO_EXTI_Callback(BTN0_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN1_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN2_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN3_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN4_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN5_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN6_Pin);
		// osDelay(100);
		// HAL_GPIO_EXTI_Callback(BTN7_Pin);
		// osDelay(100);
	}
}
void UI_task(void *argument)
{
  for(;;)
    {
		UIQueue_Msg *data;
		uint16_t voltage = 0;
		uint16_t motorCurrent = 0;
		if(osMessageQueueGet(ui_queueHandle,&data, 0, osWaitForever) == osOK){ // checks message type to update certain fields
			switch ((UIMsg)(data->MsgType)){
				case BMSRx0:
				volt = BMSFrame0::Instance().GetPackVoltage(); // Update voltage
				ui.UpdateNetPower(current * volt);
				ui.UpdateBattV(volt);
				Efficiency(); // Update Efficiency
				break;

				case BMSRx1:
				current = BMSFrame1::Instance().GetPackCurrent(); // Update Current
				ui.UpdateNetPower(current * volt);
				Efficiency(); // Update Efficiency
				break;

				case BMSRx2:
				ui.UpdateTemp(BMSFrame2::Instance().GetHighTemp());
				break;

				case BMSRx3:
				ui.UpdateSOC(BMSFrame3::Instance().GetPackSoC());
				if(BMSFrame3::Instance().GetHighChargeCurrentFault()){
					ui.DisplayError1(AUX_HIGH_CURRENT_FAULT);
				}
				else{
					ui.DisplayError1("                       ");
				}
				break;

				case PowerBoard:
				voltage = (data->data[0] << 8) | data->data[1];
				ui.UpdateAuxV(voltage);
				break;

				case MotorRx0:
				motorCurrent = ((data->data[2] & 0xF0) >> 4) | ((data->data[3] & 0x3F) << 4);
				ui.UpdateMotorPower(motorCurrent* MitsubaFrame0::Instance().GetVoltage());
				ui.UpdateSpeed(Speed(data->data)); // Update Speed
				Efficiency(); // Update Efficiency
				break;

				case MotorRx1:
				if(MitsubaFrame1::Instance().GetPowerMode()){
					ui.UpdateMode(MODE_PWR);
				}
				else{
					ui.UpdateMode(MODE_ECO);
				}
				break;

				case RightAndLeft:
				ui.ToggleLeftTurn();  
				ui.ToggleRightTurn();
				break;

				case LeftTurn:
				ui.ToggleLeftTurn();
				break;

				case RightTurn:
				ui.ToggleRightTurn();
				break;

				case Timer:
				ui.IncrementTime();
				break;

				case MPPTs:
				solarpower = SolarPower();
				ui.UpdateSolarPower(solarpower);
				break;

				default:
				break;

			}
		}

  }
}

void CAN_Init()
{
	BMSFrame0::Instance().rxCallback = UBMSRx0;
	BMSFrame1::Instance().rxCallback = UBMSRx1;
	BMSFrame2::Instance().rxCallback = UBMSRx2;
	BMSFrame3::Instance().rxCallback = UBMSRx3;
	MitsubaFrame0::Instance().rxCallback = UMotorRx0;
	MitsubaFrame1::Instance().rxCallback = UMotorRx1;
	MPPTOutputMeasurementsFrame1::Instance().rxCallback = UMPPTs;
	MPPTOutputMeasurementsFrame2::Instance().rxCallback = UMPPTs;
	MPPTOutputMeasurementsFrame3::Instance().rxCallback = UMPPTs;
	// ADdd messages to receive from different peripherals
	CANController::AddDevice(&CAN);
	CANController::AddRxMessage(&power_board);
    CANController::AddRxMessage(&MitsubaFrame0::Instance());
	CANController::AddRxMessage(&MitsubaFrame1::Instance());
	CANController::AddRxMessage(&BMSFrame0::Instance());
	CANController::AddRxMessage(&BMSFrame1::Instance());
	CANController::AddRxMessage(&BMSFrame2::Instance());
	CANController::AddRxMessage(&BMSFrame3::Instance());
	CANController::AddRxMessage(&MPPTOutputMeasurementsFrame1::Instance());
	CANController::AddRxMessage(&MPPTOutputMeasurementsFrame2::Instance());
	CANController::AddRxMessage(&MPPTOutputMeasurementsFrame3::Instance());
    CANController::AddFilterAll();
    CANController::Start();
}

/** Send UI Message during Interrupt operation for BMSRx3 */
void UBMSRx3(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx3, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for BMSRx1 */
void UBMSRx1(uint8_t data[]){
	updateEfficiency |= 0b00000100;
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx1, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for BMSRx2 */
void UBMSRx2(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx2, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for BMSRx0 */
void UBMSRx0(uint8_t data[])
{
	updateEfficiency |= 0b00000010;
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx0, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for the Motor */
void UMotorRx0(uint8_t data[])
{
	updateEfficiency |= 0b00000001;
	UIQueue_Msg *message = new UIQueue_Msg(MotorRx0, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

// Sends UI Message for the Mode of the motor: power or eco
void UMotorRx1(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(MotorRx1, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

void UMPPTs(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(MPPTs, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

void UPowerBoard(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(PowerBoard, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Calculates and updates the efficiency  */
float Efficiency(){
	if(updateEfficiency == 7){
		updateEfficiency = 0;
		float watts = current * volt;
		energy_used += ((prevwatts + watts) / 2);
		miles_driven+= ((speed + prevspeed) / 2);

		previous_time = xTaskGetTickCount();
		prevwatts = watts;
		prevspeed = speed;
		ui.UpdateEfficiency(energy_used / miles_driven);
		
		efficiency = energy_used / miles_driven;
	}
	return efficiency;	
}

// Speed Calculator
float Speed(uint8_t data[]){
	speed = ((data[4] & 0xF8 ) >> 3) | ((data[5]) & 0x7F) << 5;
	//speed = MitsubaFrame0::GetMotorRPM();
	speed *=  WHEEL_DIAM_MI * 60;
	return speed;
}

uint32_t SolarPower(){
	return (uint32_t)(MPPTOutputMeasurementsFrame1::Instance().GetOutputCurrent() * MPPTOutputMeasurementsFrame1::Instance().GetOutputVoltage()
		+  MPPTOutputMeasurementsFrame2::Instance().GetOutputCurrent() * MPPTOutputMeasurementsFrame2::Instance().GetOutputVoltage()
		+  MPPTOutputMeasurementsFrame3::Instance().GetOutputCurrent() * MPPTOutputMeasurementsFrame3::Instance().GetOutputVoltage());
}

/** Initialize all the buttons */
void Button_Init(){
	/*
	LongButton horn = LongButton(BTN7_GPIO_Port, BTN7_Pin);
	LongButton pushtotalk = LongButton(BTN8_GPIO_Port, BTN8_Pin);

	horn.release_callback_= HornReleased;
	horn.press_callback_ = HornPressed;
	pushtotalk.press_callback_ = PushToTalkPressed;
	pushtotalk.release_callback_ = PushToTalkReleased;
*/
	//BTN0.RegisterNormalPressCallback(Button0);
	//BTN1.RegisterNormalPressCallback(Button8);
	// BTN8.RegisterNormalPressCallback(Button8);
	// BTN2.RegisterNormalPressCallback(Button2);
	// BTN3.RegisterNormalPressCallback(Button3);
	// BTN4.RegisterNormalPressCallback(Button4);
	//BTN5.RegisterNormalPressCallback(Button8);
	//BTN9.RegisterNormalPressCallback(Button8);
	//BTN6.RegisterNormalPressCallback(Button6);
	

}

void Button0(){
	//Left Turn
	if(DriverControlsFrame1::Instance().GetLeftTurn()){
		DriverControlsFrame1::SetLeftTurn(false);
		if(!DriverControlsFrame1::Instance().GetHazards()){
			UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	else{
		DriverControlsFrame1::SetLeftTurn(true);
		if(!DriverControlsFrame1::Instance().GetHazards()){
			UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	CANController::Send(&DriverControlsFrame1::Instance());
}

void Button8(){
	//Right Turn
	if(DriverControlsFrame1::Instance().GetRightTurn()){
		DriverControlsFrame1::SetRightTurn(false);
		if(!DriverControlsFrame1::Instance().GetHazards()){
			UIQueue_Msg *message = new UIQueue_Msg(RightTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	else{
		DriverControlsFrame1::SetRightTurn(true);
		if(!DriverControlsFrame1::Instance().GetHazards()){
			UIQueue_Msg *message = new UIQueue_Msg(RightTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	CANController::Send(&DriverControlsFrame1::Instance());
}

/** Hazards Callback Function, checks if left and right turn were on, and toggles in UI accordingly*/
void Button2(){
	// Hazards
	if(DriverControlsFrame1::Instance().GetHazards()){
		DriverControlsFrame1::SetHazards(false);
		if(!DriverControlsFrame1::Instance().GetLeftTurn()){
			UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
		if(!DriverControlsFrame1::Instance().GetRightTurn()){
			UIQueue_Msg *message = new UIQueue_Msg(RightTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	else{
		DriverControlsFrame1::SetHazards(true);
		if(!DriverControlsFrame1::Instance().GetLeftTurn()){
			UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
		if(!DriverControlsFrame1::Instance().GetRightTurn()){
			UIQueue_Msg *message = new UIQueue_Msg(RightTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	CANController::Send(&DriverControlsFrame1::Instance());
}

void Button3(){
	//BPS Fault
	
}

void Button4(){
	//Cruise Enable
	
}

void Button5(){
	//ECO Enable
	
}

void Button6(){
	//Headlights
	if(DriverControlsFrame1::Instance().GetHeadlight()){
		DriverControlsFrame1::SetHeadlight(false);
	}
	else{
		DriverControlsFrame1::SetHeadlight(true);
	}
	CANController::Send(&DriverControlsFrame1::Instance());
}

void HornPressed(){
	// // Update UI and CAN Message
	// msg[1] |= 0b00000001;
	// message.LoadData(msg, 8);
	// CANController::SendOnDevice(&CAN, &message);
}

void HornReleased(){
	// // Update UI and CAN Message
	// msg[1] &= 0b11111110;
	// message.LoadData(msg, 8);
	// CANController::SendOnDevice(&CAN, &message);
}

void PushToTalkPressed(){
	// // Update UI and CAN Message
	// msg[4] |= 0b10000000;
	// message.LoadData(msg, 8);
	// CANController::SendOnDevice(&CAN, &message);
}

void PushToTalkReleased(){
	// // Update UI and CAN Message
	// msg[4] &= 0b01111111;
	// message.LoadData(msg, 8);
	// CANController::SendOnDevice(&CAN, &message);
}