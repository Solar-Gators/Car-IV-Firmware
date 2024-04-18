/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Authors: Taylor Gerke, Anthony Kfoury, Braden Azis
 */

#include "user.hpp"
#include "sg_can.hpp"// For CAN Messages
#include "longpressbutton.hpp"
#include "button.hpp"

#define UI_THREAD_STACK_SIZE  256
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
// FIFO for UI update messages:
const osMessageQueueAttr_t ui_queue_attr = {
	.name="UI Queue"
};

osMessageQueueId_t ui_queueHandle;
ILI9341 display(240, 320);
UI ui(&display);

uint32_t previous_time = 0;


// Can Fields
uint8_t msg[8] ={}; // data
CAN_HandleTypeDef ucan;
CANDevice CAN = CANDevice(&ucan);
CANFrame motor_rx_0 = CANFrame(0x08850225,CAN_ID_STD,CAN_RTR_DATA,8);
CANFrame bms_rx_0 = CANFrame(0x6B0, CAN_ID_STD,CAN_RTR_DATA,8);
CANFrame bms_rx_2= CANFrame(0x6B2, CAN_ID_STD, CAN_RTR_DATA,8);
CANFrame message = CANFrame(0x010,CAN_ID_STD, CAN_RTR_DATA, 8);



//LongButton setup
LongButton horn = LongButton(BTN7_GPIO_Port, BTN7_Pin);
LongButton pushtotalk = LongButton(BTN8_GPIO_Port, BTN8_Pin);

// Fields related to efficiency and speed calculation
static constexpr float WHEEL_DIAM_MI = (0.0010867658F); // wheel diameter to calculate the speed
float prevspeed = 0.001;
float prevwatts = 0;
float current = 0;
float volt = 0;
float speed = 0;
float energy_used = 0;
float miles_driven = 0;
uint8_t update = 0;
float efficiency = 0;

/** Initializes and sets up all the threads and tasks related to the UI*/
void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");

	/*Initialize UI, create CAN and UI threads, create UI Queue*/
	ui.Init();
	CAN_Init();
	Button_Init();
	ui_queueHandle = osMessageQueueNew(8, sizeof(UIQueue_Msg), &ui_queue_attr);
	ui_threadHandle = osThreadNew((osThreadFunc_t)UI_task, NULL, &ui_thread_attr);

	// Add longButton callbacks for when buttons are pressed and released
	horn.release_callback_= HornReleased;
	horn.press_callback_ = HornPressed;
	pushtotalk.press_callback_ = PushToTalkPressed;
	pushtotalk.release_callback_ = PushToTalkReleased;
	
	CAN_Callback_Init();

	HAL_Delay(100);
	ui.UpdateSpeed(35.23);
	ui.UpdateMode(MODE_PWR);

	// float speed = 1.5;


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
	// 	ui.UpdateTemp(23);// pack temp: bms rx 1 high temp
	// 	ui.UpdateEfficiency(123);//energy used (average in energy times time)/ miles driven(average speed times time)
	// 	ui.UpdateNetPower(1234);
	// 	ui.UpdateSolarPower(1234);// from MPPTs (interfaces with solar array): data from 3 and add them up and send it // use fifo: populate 
	// 	ui.UpdateMotorPower(1);
	// 	ui.DisplayError1(AUX_HIGH_CURRENT_FAULT);
	// 	ui.DisplayError2(WEAK_CELL_FAULT);
	// 	HAL_Delay(1000);
		/*TODO: use Button driver, use FIFO for MPPTS and threads for UI stuff because not important
		*/
	//}	
}

/** Initialize all the buttons */
void Button_Init(){
	Button BTN0 = Button(BTN0_GPIO_Port, BTN0_Pin, 50, GPIO_PIN_SET, false);
	Button BTN1 = Button(BTN1_GPIO_Port, BTN1_Pin, 50, GPIO_PIN_SET, false);
	Button BTN2 = Button(BTN2_GPIO_Port, BTN2_Pin, 50, GPIO_PIN_SET, false);
	Button BTN3 = Button(BTN3_GPIO_Port, BTN3_Pin, 50, GPIO_PIN_SET, false);
	Button BTN4 = Button(BTN4_GPIO_Port, BTN4_Pin, 50, GPIO_PIN_SET, false);
	Button BTN5 = Button(BTN5_GPIO_Port, BTN5_Pin, 50, GPIO_PIN_SET, false);
	Button BTN6 = Button(BTN6_GPIO_Port, BTN6_Pin, 50, GPIO_PIN_SET, false);

	BTN0.RegisterNormalPressCallback(Button0);
	BTN1.RegisterNormalPressCallback(Button1);
	BTN2.RegisterNormalPressCallback(Button2);
	BTN3.RegisterNormalPressCallback(Button3);
	BTN4.RegisterNormalPressCallback(Button4);
	BTN5.RegisterNormalPressCallback(Button5);
	BTN6.RegisterNormalPressCallback(Button6);

}
void CAN_Init()
{
	//initialize CAN
	ucan.Instance = CAN1;
 	ucan.Init.Prescaler = 5;
  	ucan.Init.Mode = CAN_MODE_NORMAL;
  	ucan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  	ucan.Init.TimeSeg1 = CAN_BS1_15TQ;
  	ucan.Init.TimeSeg2 = CAN_BS2_2TQ;
  	ucan.Init.TimeTriggeredMode = DISABLE;
  	ucan.Init.AutoBusOff = DISABLE;
  	ucan.Init.AutoWakeUp = DISABLE;
  	ucan.Init.AutoRetransmission = DISABLE;
  	ucan.Init.ReceiveFifoLocked = DISABLE;
  	ucan.Init.TransmitFifoPriority = DISABLE;
  	if (HAL_CAN_Init(&ucan) != HAL_OK)
  	{
    	Error_Handler();
	}
		
	// ADdd messages to receive from different peripherals
	CANController::AddDevice(&CAN);
    CANController::AddRxMessage(&motor_rx_0);
	CANController::AddRxMessage(&bms_rx_0);
	CANController::AddRxMessage(&bms_rx_2);
    CANController::AddFilterAll();
    CANController::Start();
}

void Button0(){
	//Left Turn
	if((msg[1] & 0b10000000)!=0){
		msg[1] &= 0b01111111;
	}
	else msg[1] |= 0b10000000;

	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
	UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

void Button1(){
	//Right Turn
	if((msg[1] & 0b01000000) != 0){
		msg[1] &= 0b1011111111;
	}
	else msg[1] |= 0b01000000;

	//send CAN message
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
	UIQueue_Msg *message= new UIQueue_Msg(RightTurn, NULL);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Hazards Callback Function, checks if left and right turn were on, and toggles in UI accordingly*/
void Button2(){
	// Hazards
	if((msg[1] & 0b00100000) != 0){
		msg[1] &= 0b11011111;
		if(((msg[1] & 0b01000000) == 0) && (msg[1] & 0b10000000) == 0 ){
			UIQueue_Msg *message = new UIQueue_Msg(RightAndLeft, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
		else if((msg[1] & 0b01000000) == 0){
			UIQueue_Msg *message = new UIQueue_Msg(RightTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
		else if((msg[1] & 0b10000000)== 0){
			UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}
	else{ 
		msg[1] |= 0b00100000;
		if((msg[1] & 0b01000000) == 0 && (msg[1] & 0b10000000) == 0){
			UIQueue_Msg *message = new UIQueue_Msg(RightAndLeft, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
		else if((msg[1] & 0b01000000) == 0){
			UIQueue_Msg *message = new UIQueue_Msg(RightTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
		else if((msg[1] & 0b10000000)== 0){
			UIQueue_Msg *message = new UIQueue_Msg(LeftTurn, NULL);
			osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
		}
	}

	//send CAN message
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void Button3(){
	//BPS Fault
	if((msg[1] & 0b00010000) !=0){
		msg[1] &= 0b11101111;
	}
	else msg[1] |= 0b00010000;
	
	//send CAN message
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void Button4(){
	//Cruise Enable
	if((msg[1] & 0b00001000) !=0){
		msg[1] &= 0b11110111;
	}
	else msg[1] |= 0b00001000;

	//send CAN message
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void Button5(){
	//ECO Enable
	if((msg[1] & 0b00000100) !=0){
		msg[1] &= 0b11111011;
	}
	else msg[1] |= 0b00000100;

	//send CAN message
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void Button6(){
	//Headlights
	if((msg[1] & 0b00000010) !=0){
		msg[1] &= 0b11111101;
	}
	else msg[1] |= 0b00000010;

	//send CAN message
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void HornPressed(){
	// Update UI and CAN Message
	msg[1] |= 0b00000001;
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void HornReleased(){
	// Update UI and CAN Message
	msg[1] &= 0b11111110;
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void PushToTalkPressed(){
	// Update UI and CAN Message
	msg[4] |= 0b10000000;
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

void PushToTalkReleased(){
	// Update UI and CAN Message
	msg[4] &= 0b01111111;
	message.LoadData(msg, 8);
	CANController::SendOnDevice(&CAN, &message);
}

/**Set callback for when CAN messages are received*/
void CAN_Callback_Init()
{
	bms_rx_0.rxCallback   =  UBMSRx0;
	motor_rx_0.rxCallback =  UMotors;
	bms_rx_2.rxCallback   =  UBMSRx2;
}

/** Send UI Message during Interrupt operation for BMSRx2 */
void UBMSRx2(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx2, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for BMSRx0 */
void UBMSRx0(uint8_t data[])
{
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx0, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for the Motor */
void UMotors(uint8_t data[])
{
	UIQueue_Msg *message = new UIQueue_Msg(MotorRx0, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Calculates and updates the efficiency  */
float Efficiency(){
	if(update <2) update++;
	else{
		update = 0;
		float watts = current * volt;
		energy_used += ((prevwatts + watts) / 2) * (((float)(xTaskGetTickCount() - previous_time)) / (float(1000 * 3600)));
		miles_driven+=((speed + prevspeed) / 2) * (((float)(xTaskGetTickCount() - previous_time)) / (float(1000 * 3600)));

		previous_time = xTaskGetTickCount();
		prevwatts = watts;
		prevspeed = speed;
		if (miles_driven >= 1.0){
			ui.UpdateEfficiency(energy_used / miles_driven);
			energy_used = 0;
			miles_driven = 0;
		}
		efficiency = energy_used / miles_driven;
	}
	return efficiency;	
}

// Speed Calculator
float Speed(uint8_t data[]){
	uint8_t small = data[4];
	uint8_t big = data[5];

	// Data shifted to get speed related data
	small <<= 3;
    big >>= 1;

    speed = small * 16;
    speed += big;
	return speed * WHEEL_DIAM_MI * 60;
}

// UI task that updates the screen according to FIFO message received
void UI_task(void *argument){
	UIQueue_Msg *data;
	for(;;){
		if(osMessageQueueGet(ui_queueHandle,&data, 0,osWaitForever) == osOK){ // checks message type to update certain fields
			switch ((UIMsg)(data->MsgType)){
				case BMSRx0:
				volt = (data->data[6] << 8 | data->data[7]) * (1e-4); // Update voltage
				Efficiency(); // Update Efficiency
				break;

				case BMSRx2:
				current=(data->data[4] << 8 | data->data[5]) / 10; // Update Current
				Efficiency(); // Update Efficiency
				break;

				case MotorRx0:
				ui.UpdateSpeed(Speed(data->data)); // Update Speed
				Efficiency(); // Update Efficiency
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

				default:
				break;

			}
		}
	}
}