/*
 * user.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: Taylor Gerke
 */

#include "user.hpp"
#include "sg_can.hpp"
#define UI_THREAD_STACK_SIZE  256

extern "C" void CPP_UserSetup(void);
extern "C" CAN_HandleTypeDef hcan1;

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


ILI9341 display(240, 320);
UI ui(&display);
uint32_t previous_time = 0;

// Can Fields
uint8_t msg[8] ={}; // data
CANDevice CAN = CANDevice(&hcan1);
CANFrame motor_rx_0 = CANFrame(0x08850225,CAN_ID_EXT,CAN_RTR_DATA,8, UMotors);
CANFrame bms_rx_0 = CANFrame(0x6B0, CAN_ID_STD,CAN_RTR_DATA,8, UBMSRx0);
CANFrame bms_rx_2= CANFrame(0x6B2, CAN_ID_STD, CAN_RTR_DATA,8, UBMSRx2);
CANFrame bms_rx_4= CANFrame(0x6B4, CAN_ID_STD, CAN_RTR_DATA,8, UBMSRx4);
CANFrame message = CANFrame(0x010,CAN_ID_STD, CAN_RTR_DATA, 8);


// Fields related to efficiency and speed calculation
static constexpr float WHEEL_DIAM_MI = (0.0010867658F); // wheel diameter in miles / rotation
float prevspeed = 0.001;
float prevwatts = 0;
float current = 0;
float volt = 0;
float speed = 1.5;
float energy_used = 0;
float miles_driven = 0;
uint8_t update = 0;
float efficiency = 0;


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
osMessageQueueId_t ui_queueHandle;
// FIFO for UI update messages:
const osMessageQueueAttr_t ui_queue_attr = {
	.name="UI Queue"
};



void CPP_UserSetup() {
	Logger::LogInfo("Entered user setup\n");
	ui.Init();
	ui.UpdateSpeed(35.23);
	ui.UpdateMode(MODE_PWR);
	osKernelInitialize();
	ui_queueHandle = osMessageQueueNew(8, sizeof(UIQueue_Msg), &ui_queue_attr);
	ui_threadHandle = osThreadNew(UI_task, NULL, &ui_thread_attr);
	CAN_Init();
}
void UI_task(void *argument)
{
  for(;;)
    {
// 	uint8_t dt[8] = {
// 		0,0,0,0,0b00001011, 0b11111010,0,0
// 	};
// 	motor_rx_0.LoadData(dt,8);
// 	CANController::SendOnDevice(&CAN, &motor_rx_0);
// 	osDelay(2000);
// 	dt[7] = 100000;
// 	bms_rx_0.LoadData(dt,8);
// 	CANController::SendOnDevice(&CAN, &bms_rx_0);

// 	osDelay(2000);
// 	dt[5] = 100;
// 	bms_rx_2.LoadData(dt,8);
// 	CANController::SendOnDevice(&CAN, &bms_rx_2);

// 	osDelay(2000);
// 	dt[3] = 75;
// 	bms_rx_4.LoadData(dt,8);
// 	CANController::SendOnDevice(&CAN, &bms_rx_4);
	UIQueue_Msg *data;
	osDelay(2000);
	if(osMessageQueueGet(ui_queueHandle,&data, 0, osWaitForever) == osOK){ // checks message type to update certain fields
		switch ((UIMsg)(data->MsgType)){
			case BMSRx0:
			volt = (data->data[6] << 8 | data->data[7]) * (1e-4); // Update voltage
			ui.UpdateBattV(volt);
			Efficiency(); // Update Efficiency
			break;

			case BMSRx2:
			current = (data->data[4] << 8 | data->data[5]) / 10; // Update Current
			Efficiency(); // Update Efficiency
			break;

			case BMSRx4:
			ui.UpdateSOC(data->data[3]);
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

void CAN_Init()
{
	// ADdd messages to receive from different peripherals
	CANController::AddDevice(&CAN);
    CANController::AddRxMessage(&motor_rx_0);
	CANController::AddRxMessage(&bms_rx_0);
	CANController::AddRxMessage(&bms_rx_2);
	CANController::AddRxMessage(&bms_rx_4);
    CANController::AddFilterAll();
    CANController::Start();
}

/** Send UI Message during Interrupt operation for BMSRx4 */
void UBMSRx4(uint8_t data[]){
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx4, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for BMSRx2 */
void UBMSRx2(uint8_t data[]){
	update |= 0b00000100;
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx2, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for BMSRx0 */
void UBMSRx0(uint8_t data[])
{
	update |= 0b00000010;
	UIQueue_Msg *message = new UIQueue_Msg(BMSRx0, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Send UI Message during Interrupt operation for the Motor */
void UMotors(uint8_t data[])
{
	update |= 0b00000001;
	UIQueue_Msg *message = new UIQueue_Msg(MotorRx0, data);
	osMessageQueuePut(ui_queueHandle, &message, 0, 2000);
}

/** Calculates and updates the efficiency  */
float Efficiency(){
	if(update == 7){
		update = 0;
		float watts = current * volt;
		energy_used += ((prevwatts + watts) / 2) * (((float)(xTaskGetTickCount() - previous_time)) / (float(1000 * 3600)));
		miles_driven+=((speed + prevspeed) / 2) * (((float)(xTaskGetTickCount() - previous_time)) / (float(1000 * 3600)));

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
	uint8_t small = data[4];
	uint8_t big = data[5];

	// Data shifted to get speed related data
	small <<= 3;
    big >>= 1;

    speed = small * 16;
    speed += big;
	return speed * WHEEL_DIAM_MI * 60;
}
