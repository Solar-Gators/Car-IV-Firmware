#include "user.hpp"
#include "longpressbutton.hpp"
#include "button.hpp"

/* In this program, two tasks are generated. One is triggered by a periodic timer, which
 * expires every 500ms. The other is a regular thread, which is triggered by and event flag.
 * The periodic timer task sets the event flag, which causes the regular thread to exit
 * its wait state and execute. The regular thread then toggles the LED and prints a message.
 * 
 * CPP_UserSetup is called from main.cpp, and is where the user should put their setup code.
 * It is run before the RTOS scheduler is started.
*/

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

void PressCallback();
void ReleaseCallback();
void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    LongButton button1 = LongButton(BTN1_GPIO_Port,BTN1_Pin);
    button1.press_callback_ = PressCallback;
    button1.release_callback_ = ReleaseCallback;
    regular_task_id = osThreadNew((osThreadFunc_t)RegularTask1, NULL, &regular_task_attributes);
    osTimerStart(periodic_timer_id, 1000);
}

void PeriodicTask1(void *argument) {
    Logger::LogInfo("Periodic timer fired\n");
    osEventFlagsSet(regular_event, 0x1);
}

void PressCallback(){
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_SET);
}
void ReleaseCallback(){
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_RESET);
}

void RegularTask1(void *argument) {
    while (1) {
        osEventFlagsWait(regular_event, 0x1, osFlagsWaitAny, osWaitForever);

        Logger::LogInfo("Hello World!\n");
        HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
    }
}