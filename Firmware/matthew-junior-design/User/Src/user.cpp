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

extern "C" SPI_HandleTypeDef hspi1;

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

uint8_t frame_buffer_[153600];

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    
    for (uint32_t i = 0; i < 153600; i++)
        frame_buffer_[i] = i % 256;

    for (uint32_t i = 0; i < 153600; i++)
        HAL_SPI_Transmit(&hspi1, &frame_buffer_[i], 1, HAL_MAX_DELAY);

    // while (1) {
    //     uint8_t data = 0x12;
    //     HAL_GPIO_WritePin(TFTCS_GPIO_Port, TFTCS_Pin, GPIO_PIN_RESET);
    //     HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    //     HAL_GPIO_WritePin(TFTCS_GPIO_Port, TFTCS_Pin, GPIO_PIN_SET);
    //     HAL_Delay(1);
    // }

    ST7789 display = ST7789(&hspi1, TFTCS_GPIO_Port, TFTCS_Pin, 
                            TFTDC_GPIO_Port, TFTDC_Pin, 0, 0, 0, 0, frame_buffer_);

    display.Init();
    

    while (1) {
        display.Fill(RGB565_RED);
        display.Fill(RGB565_BLACK);
    }

    regular_task_id = osThreadNew((osThreadFunc_t)RegularTask1, NULL, &regular_task_attributes);
    osTimerStart(periodic_timer_id, 1000);
}

void PeriodicTask1(void *argument) {
    Logger::LogInfo("Periodic timer fired\n");
    osEventFlagsSet(regular_event, 0x1);
}

void RegularTask1(void *argument) {
    while (1) {
        osEventFlagsWait(regular_event, 0x1, osFlagsWaitAny, osWaitForever);

        Logger::LogInfo("Hello World!\n");
        HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
    }
}