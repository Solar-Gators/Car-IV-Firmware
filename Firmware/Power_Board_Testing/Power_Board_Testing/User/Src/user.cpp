#include "user.hpp"

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
void PeriodicTask2(void *argument);
void RegularTask1(void *argument);
void RegularTask2(void *argument);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* Periodic timer definitions */
osTimerAttr_t periodic_timer_attr = {
    .name = "Periodic Task 1",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_timer_id = osTimerNew((osThreadFunc_t)PeriodicTask1, osTimerPeriodic, NULL, &periodic_timer_attr);

/* Fast blink timer definitions */
osTimerAttr_t fast_blink_timer_attr = {
    .name = "Fast Blink Task",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t fast_blink_timer_id = osTimerNew((osThreadFunc_t)PeriodicTask2, osTimerPeriodic, NULL, &fast_blink_timer_attr);

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

/* Fast blink task definitions */
osThreadId_t fast_blink_task_id;
uint32_t fast_blink_task_buffer[128];
StaticTask_t fast_blink_task_control_block;
const osThreadAttr_t fast_blink_task_attributes = {
    .name = "Fast Blink Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &fast_blink_task_control_block,
    .cb_size = sizeof(fast_blink_task_control_block),
    .stack_mem = &fast_blink_task_buffer[0],
    .stack_size = sizeof(fast_blink_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};

/* Event flag to trigger regular task */
osEventFlagsId_t regular_event = osEventFlagsNew(NULL);
osEventFlagsId_t fast_event = osEventFlagsNew(NULL);


void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    //lcd_init(); //initialize LCD

    regular_task_id = osThreadNew((osThreadFunc_t)RegularTask1, NULL, &regular_task_attributes);
    fast_blink_task_id = osThreadNew((osThreadFunc_t)RegularTask2, NULL, &fast_blink_task_attributes);
    osTimerStart(periodic_timer_id, 1000);
    osTimerStart(fast_blink_timer_id, 200);

}

void PeriodicTask1(void *argument) {
    Logger::LogInfo("Periodic timer fired\n");
    osEventFlagsSet(regular_event, 0x1);
}

void PeriodicTask2(void *argument) {
    Logger::LogInfo("Periodic timer  w fired\n");
    osEventFlagsSet(fast_event, 0x1);
}

void RegularTask1(void *argument) {
    
    //HAL_TIM_Base_Start_IT(&htim6);
    
    while (1) {
        osEventFlagsWait(regular_event, 0x1, osFlagsWaitAny, osWaitForever);
        HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin); 
        Logger::LogInfo("Hello World!\n");
        
    }
}

void RegularTask2(void *argument) {

     while (1) {
        osEventFlagsWait(fast_event, 0x1, osFlagsWaitAny, osWaitForever);
        HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

    }
}

// // this is the callback for when any of the buttons is pressed.
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//     //switch (GPIO_Pin) {
  
    
// }


// // audio output funciton for 24000Khz audio signal
// void TimerSixCallback(void)
// {
//     //for testing, current_waveform is sine
//     // static uint8_t index = 0;

// }
