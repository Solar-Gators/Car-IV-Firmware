#include "user.hpp"
#include "IMU.h"
#include "usbd_cdc_if.h"
#include <string>
#include "IoTestFrame.hpp"
#include "MotorControlFrame.hpp"

using namespace std;

/* In this program, two tasks are generated. One is triggered by a periodic timer, which
 * expires every 500ms. The other is a regular thread, which is triggered by and event flag.
 * The periodic timer task sets the event flag, which causes the regular thread to exit
 * its wait state and execute. The regular thread then toggles the LED and prints a message.
 * 
 * CPP_UserSetup is called from main.cpp, and is where the user should put their setup code.
 * It is run before the RTOS scheduler is started.
*/
extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;


/* Initialize CAN frames and devices */

CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);


extern "C" void CPP_UserSetup(void);

/* Task function prototypes */
void PeriodicTask1(void *argument);
void RegularTask1(void *argument);

/* Periodic timer definitions */
osTimerAttr_t periodic_timer_attr = {
    .name = "Periodic Task 1",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_timer_id = osTimerNew((osThreadFunc_t)PeriodicTask1, osTimerPeriodic, NULL, &periodic_timer_attr);

/* Regular task definitions */
osThreadId_t regular_task_id;
uint32_t regular_task_buffer[128];
StaticTask_t regular_task_control_block;
const osThreadAttr_t regular_task_attributes = {
    .name = "Regular Task 1",
    .attr_bits = osThreadDetached,
    .cb_mem = &regular_task_control_block,
    .cb_size = sizeof(regular_task_control_block),
    .stack_mem = &regular_task_buffer[0],
    .stack_size = sizeof(regular_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};

/* Event flag to trigger regular task */
osEventFlagsId_t regular_event = osEventFlagsNew(NULL);

IMU imu;
uint8_t txBuffer[3];
string text;
float z_accel;
uint16_t raw;
uint8_t high;
uint8_t low;


void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly

    
    IMU_INIT(&imu, &hi2c2);

    HAL_Delay(10);

    regular_task_id = osThreadNew((osThreadFunc_t)RegularTask1, NULL, &regular_task_attributes);
    osTimerStart(periodic_timer_id, 20);

    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    //CANController::AddRxMessage(&io_test_frame, IoMsgCallback);
    CANController::AddFilterAll();
    CANController::Start();
}

void PeriodicTask1(void *argument) {
    //heart beat
    //Logger::LogInfo("Periodic timer fired\n");
    HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);

    //IMU_ReadAccel(&imu);
    volatile HAL_StatusTypeDef status = HAL_OK;
    status = HAL_ADC_Start(&hadc2);
	status = HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc2);

    high = (uint8_t)((raw & 0xFF00) >> 8);
    low = (uint8_t)(raw & 0x00FF);
    txBuffer[0] = high;
    txBuffer[1] = low; 
    txBuffer[2] = '\n'; 
	Logger::LogInfo("raw value: %x\n", raw);

    DriverControlsFrame0::SetThrottleVal((uint16_t)(raw) << 4);
    CANController::Send(&DriverControlsFrame0::Instance());

    osEventFlagsSet(regular_event, 0x1);
}

void RegularTask1(void *argument) {
    while (1) {
        osEventFlagsWait(regular_event, 0x1, osFlagsWaitAny, osWaitForever);

       // Logger::LogInfo("Hello World!\n");
        
        
    }
}


