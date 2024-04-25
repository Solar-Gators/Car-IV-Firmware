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

osTimerAttr_t update_ui_periodic_timer_attr = {
    .name = "Read Buttons Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t update_ui_periodic_timer_id = osTimerNew((osThreadFunc_t)UpdateUIPeriodic, 
                                                            osTimerPeriodic, 
                                                            NULL, 
                                                            &update_ui_periodic_timer_attr);

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

/* Event flags */
osEventFlagsId_t turn_signal_event = osEventFlagsNew(NULL);

static GPIO_PinState cruise_minus_last_state = GPIO_PIN_SET;
static GPIO_PinState pv_last_state = GPIO_PIN_SET;
static GPIO_PinState horn_last_state = GPIO_PIN_SET;
static GPIO_PinState ptt_last_state = GPIO_PIN_SET;
void ReadButtonsPeriodic() {
    // Check if cruise minus is pressed
    if (cruise_minus_btn.ReadPin() != cruise_minus_last_state) {
        Button::triggered_button_ = &cruise_minus_btn;
        osSemaphoreRelease(Button::button_semaphore_id_);
    }

    // Check if PV is pressed
    if (pv_btn.ReadPin() != pv_last_state) {
        Button::triggered_button_ = &pv_btn;
        osSemaphoreRelease(Button::button_semaphore_id_);
    }

    // Check held buttons (horn, PTT)
    if (horn_btn.ReadPin() != horn_last_state) {
        DriverControlsFrame1::Instance().SetHorn(static_cast<bool>(horn_btn.ReadPin()));
        CANController::Send(&DriverControlsFrame1::Instance());        
    }

    if (ptt_btn.ReadPin() != ptt_last_state) {
        DriverControlsFrame1::Instance().SetPTT(static_cast<bool>(ptt_btn.ReadPin()));
        CANController::Send(&DriverControlsFrame1::Instance());
    }
}

/* Mutexes */
osMutexId_t ui_mutex = osMutexNew(NULL);

void UpdateUIPeriodic() {
    // Wheel diameter in miles/rotation
    static constexpr float WHEEL_DIAM_MI = (0.0010867658F);

    osMutexAcquire(ui_mutex, osWaitForever);

    // Update speed
    ui.UpdateSpeed(MitsubaFrame0::Instance().GetMotorRPM() * WHEEL_DIAM_MI * 60.0F);

    // Update SoC
    ui.UpdateSOC(static_cast<float>(BMSFrame3::Instance().GetPackSoC()));

    // Update battery voltage
    ui.UpdateBattV(static_cast<float>(BMSFrame0::Instance().GetPackVoltage()) * 100.0);

    // Update battery temperature
    ui.UpdateTemp(BMSFrame2::Instance().GetHighTemp() * 100);

    // Update net power
    ui.UpdateNetPower(BMSFrame1::Instance().GetAveragePower());

    osMutexRelease(ui_mutex);
}

void TurnSignalToggle() {
    while (1) {
        if (right_turn_btn.GetToggleState() == true) {
            osMutexAcquire(ui_mutex, osWaitForever);
            ui.ToggleRightTurn();
            ui.DeactivateLeftTurn();
            osMutexRelease(ui_mutex);
            osDelay(500);
        }
        else if (left_turn_btn.GetToggleState() == true) {
            osMutexAcquire(ui_mutex, osWaitForever);
            ui.ToggleLeftTurn();
            ui.DeactivateRightTurn();
            osMutexRelease(ui_mutex);
            osDelay(500);
        }
        else {
            osMutexAcquire(ui_mutex, osWaitForever);
            ui.DeactivateRightTurn();
            ui.DeactivateLeftTurn();
            osMutexRelease(ui_mutex);
            osEventFlagsWait(turn_signal_event, 0x1, osFlagsWaitAny, osWaitForever);
        }
    }
}

void LeftTurnCallback() {
    Logger::LogInfo("Left turn pressed");
    right_turn_btn.SetToggleState(false);
    DriverControlsFrame1::Instance().SetLeftTurn(left_turn_btn.GetToggleState());
    DriverControlsFrame1::Instance().SetRightTurn(right_turn_btn.GetToggleState());
    CANController::Send(&DriverControlsFrame1::Instance());
    osEventFlagsSet(turn_signal_event, 0x1);
}

static SteeringModeTypeDef mode = MODE_ECO;
static SteeringModeTypeDef last_fwd_mode = MODE_ECO;
void ModeCallback() {
    Logger::LogInfo("Mode pressed");
    if (mode == MODE_REV)
        return;
    else if (mode == MODE_PWR)
        mode = MODE_ECO;
    else if (mode == MODE_ECO)
        mode = MODE_PWR;
    SendMode(mode);
}

void ModeLongCallback() {
    Logger::LogInfo("Mode long pressed");
    if (mode == MODE_ECO) {
        mode = MODE_REV;
        last_fwd_mode = MODE_ECO;
    }
    else if (mode == MODE_PWR) {
        mode = MODE_REV;
        last_fwd_mode = MODE_PWR;
    }
    else if (mode == MODE_REV) {
        mode = last_fwd_mode;
    }
    SendMode(mode);
}

void RegenCallback() {
    Logger::LogInfo("Regen pressed");
}

void HornCallback() {
    Logger::LogInfo("Horn pressed");
}

void MCCallback() {
    Logger::LogInfo("MC pressed");
    DriverControlsFrame1::Instance().SetMotorEnable(mc_btn.GetToggleState());
    osMutexAcquire(ui_mutex, osWaitForever);
    if (mc_btn.GetToggleState() == false)
        ui.UpdateMCStatus(RGB565_RED);
    else
        ui.UpdateMCStatus(RGB565_GREEN);
    osMutexRelease(ui_mutex);
}

void RightTurnCallback() {
    Logger::LogInfo("Right turn pressed");
    left_turn_btn.SetToggleState(false);
    DriverControlsFrame1::Instance().SetRightTurn(right_turn_btn.GetToggleState());
    DriverControlsFrame1::Instance().SetLeftTurn(left_turn_btn.GetToggleState());
    CANController::Send(&DriverControlsFrame1::Instance());
    osEventFlagsSet(turn_signal_event, 0x1);
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

void PVCallback() {
    Logger::LogInfo("PV pressed");
}

void BMSFrame3Callback(uint8_t *data) {
    uint8_t fault_flags = BMSFrame3::Instance().GetFaultFlags();
    osMutexAcquire(ui_mutex, osWaitForever);
    if (fault_flags)
        ui.UpdateBMSStatus(RGB565_RED);
    else
        ui.UpdateBMSStatus(RGB565_GREEN);
    osMutexRelease(ui_mutex);
}

void ThreadsStart() {
    // Read button state every 20 ms
    osTimerStart(read_buttons_periodic_timer_id, 20);

    // Update UI every 500ms
    osTimerStart(update_ui_periodic_timer_id, 500);
}