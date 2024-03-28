#include "threads.hpp"

#include "usbd_cdc_if.h"
#include "etl/string.h"

/* Externs */
extern Button user_button;
extern CANFrame io_msg;
extern CANDevice candev1;
extern CANDevice candev2;

/* Periodic thread definitions */
osTimerAttr_t periodic_thread_attr = {
    .name = "Periodic Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_thread_id = osTimerNew((osTimerFunc_t)PeriodicThread, osTimerPeriodic, NULL, &periodic_thread_attr);

/* Thread definitions */
osThreadId_t command_thread_id;
uint32_t command_thread_buffer[128];
StaticTask_t command_thread_control_block;
const osThreadAttr_t command_thread_attributes = {
    .name = "Command Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &command_thread_control_block,
    .cb_size = sizeof(command_thread_control_block),
    .stack_mem = &command_thread_buffer[0],
    .stack_size = sizeof(command_thread_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};

void ThreadsInit() {
    // Start command thread
    command_thread_id = osThreadNew((osThreadFunc_t)ProcessCommand, NULL, &command_thread_attributes);

    // Start heartbeat thread
    osTimerStart(periodic_thread_id, 500);
}

/* Thread function definitions */
void PeriodicThread(void *argument) {
    Logger::LogInfo("Still alive!\n");
}

void ProcessCommand(void* argument) {
    // Command buffer
    static char command_buf[CMD_MAX_SIZE];

    while (1) {
        // Get command from queue
        osMessageQueueGet(command_queue, command_buf, NULL, osWaitForever);
        Logger::LogInfo("Received command: %s\n", command_buf);

        // Get command token
        char* command_token = strtok(command_buf, " ");

        // Call command based on token
        if (command_map.find(etl::string<TOKEN_MAX_SIZE>(command_token)) != command_map.end()) {
            command_map.at(etl::string<TOKEN_MAX_SIZE>(command_token))();
        } else {
            VCOM_Transmit((const char*)"Command not found\n", 18);
        }
    }
}

/* Callbacks */
void ButtonSingleCallback(void) {
    Logger::LogInfo("Button Single Press\n");
    //io_msg.data[0] = user_button.GetToggleState();
    IoTestFrame::SetOkLed((GPIO_PinState)user_button.GetToggleState());
    CANController::Send(&IoTestFrame::Instance());
}

void ButtonLongCallback(void) {
    Logger::LogInfo("Button Long Press\n");
    IoTestFrame::SetErrorLed((GPIO_PinState)user_button.GetLongToggleState());
    CANController::Send(&IoTestFrame::Instance());
}

void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, IoTestFrame::GetOkLed());
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, IoTestFrame::GetErrorLed());
}

void MitsubaMsgCallback(uint8_t *data) {
    // Do something with Mitsuba message
}