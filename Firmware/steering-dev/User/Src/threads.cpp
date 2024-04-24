#include "threads.hpp"

/* Global data */

/* Setup periodic threads */
osTimerAttr_t read_buttons_periodic_timer_attr = {
    .name = "Read Buttons Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t read_buttons_periodic_timer_id = osTimerNew((osThreadFunc_t)ReadButtonsPeriodic, 
                                                            osTimerPeriodic, 
                                                            NULL, 
                                                            &read_buttons_periodic_timer_attr);

/* Setup regular threads */
uint32_t turn_signal_thread_buffer[128];
StaticTask_t turn_signal_thread_control_block;
const osThreadAttr_t turn_signal_thread_attributes = {
    .name = "Logging Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &turn_signal_thread_control_block,
    .cb_size = sizeof(turn_signal_thread_control_block),
    .stack_mem = &turn_signal_thread_buffer[0],
    .stack_size = sizeof(turn_signal_thread_buffer),
    .priority = (osPriority_t) osPriorityBelowNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t logger_thread_id = osThreadNew((osThreadFunc_t)TurnSignalToggle, 
                                            NULL, 
                                            &turn_signal_thread_attributes);

static GPIO_PinState right_turn_last_state = GPIO_PIN_SET;

void ReadButtonsPeriodic() {
    // Check if right turn is pressed
    if (right_turn_btn.ReadPin() != right_turn_last_state) {
        right_turn_last_state = right_turn_btn.ReadPin();
        Button::triggered_button_ = &right_turn_btn;
        osSemaphoreRelease(Button::button_semaphore_id_);
    }
}

void TurnSignalToggle() {
    while (1) {
        if (right_turn_btn.GetToggleState() == true) {
            ui.ToggleRightTurn();
        } else {
            ui.DeactivateRightTurn();
        }

        osDelay(500);
    }
}

void RightTurnCallback() {
    Logger::LogInfo("Right turn pressed");
    ui.ToggleRightTurn();
}

void CruisePlusCallback() {
    Logger::LogInfo("Cruise plus pressed");
}

void CruiseMinusCallback() {
    Logger::LogInfo("Cruise minus pressed");
}

void PTTCallback() {
    Logger::LogInfo("PTT pressed");
}

void ThreadsStart() {
    // Read button state every 20 ms
    osTimerStart(read_buttons_periodic_timer_id, 20);
}