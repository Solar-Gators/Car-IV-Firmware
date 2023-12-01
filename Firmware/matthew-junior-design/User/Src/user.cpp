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

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    HAL_GPIO_WritePin(TFTBL_GPIO_Port, TFTBL_Pin, GPIO_PIN_SET);

    ST7789 display = ST7789(&hspi1, TFTCS_GPIO_Port, TFTCS_Pin, 
                            TFTDC_GPIO_Port, TFTDC_Pin, 0, 0, 0, 0);

    display.Init();

    while (1) {
        display.Fill(ST7789_WHITE);
        display.DrawText(&FontStyle_Emulogic, "How happy is the little stone", 30, 200, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "That rambles in the road alone,", 30, 180, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "And doesn't care about careers,", 30, 160, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "And exigencies never fears;", 30, 140, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "Whose coat of elemental brown", 30, 120, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "A passing universe put on;", 30, 100, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "And independent as the sun,", 30, 80, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "Associates or grows alone,", 30, 60, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "Fulfilling absolute decree", 30, 40, ST7789_BLACK);
        display.DrawText(&FontStyle_Emulogic, "In casual simplicity", 30, 20, ST7789_BLACK);
        HAL_Delay(500);
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