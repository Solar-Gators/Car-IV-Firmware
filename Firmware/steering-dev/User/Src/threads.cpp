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
    // if (cruise_minus_btn.ReadPin() != cruise_minus_last_state) {
    //     Button::triggered_button_ = &cruise_minus_btn;
    //     osSemaphoreRelease(Button::button_semaphore_id_);
    // }

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
static uint32_t solar_power = 0;
void UpdateUIPeriodic() {
    // Wheel diameter in miles/rotation
    static constexpr float WHEEL_DIAM_MI = (0.0010867658F);

    osMutexAcquire(ui_mutex, osWaitForever);

    // Update speed
    ui.UpdateSpeed(MitsubaFrame0::Instance().GetMotorRPM() * WHEEL_DIAM_MI * 60.0F);

    // Update battery voltage
    ui.UpdateBattV(static_cast<float>(BMSFrame0::Instance().GetPackVoltage()) / 100.0);

    // Update battery temperature
    ui.UpdateTemp(BMSFrame2::Instance().GetHighTemp() / 100.0);

    // Update net power
    ui.UpdateNetPower(BMSFrame1::Instance().GetAveragePower());

    // Update solar power
    uint32_t solar_power_old = solar_power;
    solar_power = static_cast<uint32_t>(MPPTInputMeasurementsFrame1::Instance().GetInputCurrent() * MPPTInputMeasurementsFrame1::Instance().GetInputVoltage());
    solar_power += static_cast<uint32_t>(MPPTInputMeasurementsFrame2::Instance().GetInputCurrent() * MPPTInputMeasurementsFrame2::GetInputVoltage());
    solar_power += static_cast<uint32_t>(MPPTInputMeasurementsFrame3::Instance().GetInputCurrent() * MPPTInputMeasurementsFrame3::GetInputVoltage());
    ui.UpdateSolarPower(solar_power);

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

void BMSResetCallback() {
    Logger::LogInfo("BMS reset pressed");
    DriverControlsFrame1::Instance().SetBMSReset(true);
    CANController::Send(&DriverControlsFrame1::Instance());
    osDelay(10); // Delay to allow message to send
    DriverControlsFrame1::Instance().SetBMSReset(false);
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

// TODO: Switch to PV button
void PVCallback() {
    Logger::LogInfo("PV pressed");
    bool pv_state = pv_btn.GetToggleState();
    DriverControlsFrame1::Instance().SetPVEnable(pv_state);
    CANController::Send(&DriverControlsFrame1::Instance());
}

void BMSFrame3Callback(uint8_t *data) {
    // For some reason, frame will be all 0s sometimes
    // TODO: Figure out why this is the case
    if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0)
        return;

    // Update BMS status
    uint8_t fault_flags = BMSFrame3::Instance().GetFaultFlags();
    osMutexAcquire(ui_mutex, osWaitForever);
    if (fault_flags)
        ui.UpdateBMSStatus(RGB565_RED);
    else
        ui.UpdateBMSStatus(RGB565_GREEN);

    // Display error text
    // If multiple errors are present, only the first one will be displayed
    if (BMSFrame3::Instance().GetHighDischargeCurrentFault())
        ui.DisplayError1("High Discharge Current");
    else if (BMSFrame3::Instance().GetHighChargeCurrentFault())
        ui.DisplayError1("High Charge Current");
    else if (BMSFrame3::Instance().GetHighCellVoltageFault())
        ui.DisplayError1("High Cell Voltage");
    else if (BMSFrame3::Instance().GetLowCellVoltageFault())
        ui.DisplayError1("Low Cell Voltage");
    else if (BMSFrame3::Instance().GetHighTempFault())
        ui.DisplayError1("High Cell Temp");
    else
        ui.DisplayError1("");

    // Update SoC
    ui.UpdateSOC(static_cast<float>(BMSFrame3::Instance().GetPackSoC()));

    // Update PV status
    if (BMSFrame3::Instance().GetContactorStatus(1))
        ui.UpdatePVStatus(RGB565_GREEN);
    else
        ui.UpdatePVStatus(RGB565_RED);
    osMutexRelease(ui_mutex);
}

void PowerBoardCallback(uint8_t *data) {
    // Update supp batt voltage
    osMutexAcquire(ui_mutex, osWaitForever);
    ui.UpdateAuxV(static_cast<float>(PowerBoardFrame::GetSuppBattVoltage()) / 100.0);
    osMutexRelease(ui_mutex);
}

void ThreadsStart() {
    // Read button state every 20 ms
    osTimerStart(read_buttons_periodic_timer_id, 20);

    // Update UI every 500ms
    osTimerStart(update_ui_periodic_timer_id, 500);
}