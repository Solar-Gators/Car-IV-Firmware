#include "threads.h"

#include "etl/to_string.h"
#include "etl/format_spec.h"
#include "etl/string_utilities.h"

/* Setup data structures */
osEventFlagsId_t log_event = osEventFlagsNew(NULL);

/* Setup periodic threads */
osTimerAttr_t mitsuba_req_periodic_timer_attr = {
    .name = "Send Mitsuba Request Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t mitsuba_req_periodic_timer_id = osTimerNew((osThreadFunc_t)SendMitsubaRequest, 
                                                            osTimerPeriodic, 
                                                            NULL, 
                                                            &mitsuba_req_periodic_timer_attr);

osTimerAttr_t toggle_lights_periodic_timer_attr = {
    .name = "Toggle Lights Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t toggle_lights_periodic_timer_id = osTimerNew((osThreadFunc_t)ToggleLights, 
                                                            osTimerPeriodic, 
                                                            NULL, 
                                                            &toggle_lights_periodic_timer_attr);

osTimerAttr_t logger_periodic_timer_attr = {
    .name = "Toggle Lights Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t logger_periodic_timer_id = osTimerNew((osThreadFunc_t)LogDataPeriodic, 
                                                            osTimerPeriodic, 
                                                            NULL, 
                                                            &logger_periodic_timer_attr);

/* Setup regular threads */
uint32_t logger_thread_buffer[2048];
StaticTask_t logger_thread_control_block;
const osThreadAttr_t logger_thread_attributes = {
    .name = "Logging Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &logger_thread_control_block,
    .cb_size = sizeof(logger_thread_control_block),
    .stack_mem = &logger_thread_buffer[0],
    .stack_size = sizeof(logger_thread_buffer),
    .priority = (osPriority_t) osPriorityBelowNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t logger_thread_id = osThreadNew((osThreadFunc_t)LogData, NULL, &logger_thread_attributes);

/* Start periodic threads */
void ThreadsStart() {
    // Request to receive Mitsuba frames every 500ms
    osTimerStart(mitsuba_req_periodic_timer_id, 500);

    // Toggle lights (hazards & turn signals) every 500ms
    osTimerStart(toggle_lights_periodic_timer_id, 500);

    // Toggle logger thread every 100ms
    osTimerStart(logger_periodic_timer_id, 100);
}

/* Periodic thread function to send CAN message request for frames to Mitsuba 
    Also toggle heartbeat LED */
void SendMitsubaRequest() {
    // Send a request to the Mitsuba motor controller
    CANController::Send(&MitsubaRequestFrame::Instance());

    // Heartbeat LED
    HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
}

/* Periodic thread function to handle hazards and turn signal logic */
void ToggleLights() {
    // If hazards or left turn on, toggle left lights
    if (DriverControlsFrame1::GetHazards() || DriverControlsFrame1::GetLeftTurn()) {
        HAL_GPIO_TogglePin(FL_LIGHT_EN_GPIO_Port, FL_LIGHT_EN_Pin);
        HAL_GPIO_TogglePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin);
    }
    // If neither hazards nor left turn on, left lights depend on brake lights
    else {
        HAL_GPIO_WritePin(FL_LIGHT_EN_GPIO_Port, FL_LIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin, 
                            static_cast<GPIO_PinState>(DriverControlsFrame0::GetBrakeEnable()));
    }
    // If hazards or right turn on, toggle right lights
    if (DriverControlsFrame1::GetHazards() || DriverControlsFrame1::GetRightTurn()) {
        HAL_GPIO_TogglePin(FR_LIGHT_EN_GPIO_Port, FR_LIGHT_EN_Pin);
        HAL_GPIO_TogglePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin);
    }
    // If neither hazards nor right turn on, right lights depend on brake lights
    else {
        HAL_GPIO_WritePin(FR_LIGHT_EN_GPIO_Port, FR_LIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin, 
                            static_cast<GPIO_PinState>(DriverControlsFrame0::GetBrakeEnable()));
    }

    // If brake lights are not on, center light turns off
    if (!DriverControlsFrame0::GetBrakeEnable()) {
        HAL_GPIO_WritePin(RC_LIGHT_EN_GPIO_Port, RC_LIGHT_EN_Pin, GPIO_PIN_RESET);
    }

    // If in kill state or BMS trip, turn on strobe light, otherwise off
    if (kill_state || bms_trip)
        HAL_GPIO_WritePin(STRB_LIGHT_EN_GPIO_Port, STRB_LIGHT_EN_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(STRB_LIGHT_EN_GPIO_Port, STRB_LIGHT_EN_Pin, GPIO_PIN_RESET);
}

/* 
 * Periodic thread function to unblock SD logging thread
 * Logging is not done in this periodic thread directly so the SD logging
 * thread can be set to a lower priority than the timer task and so the stack
 * size of the logging thread is configurable
 */
void LogDataPeriodic() {
    osEventFlagsSet(log_event, 0x1);
}

/* Thread function to log data to SD card */
void LogData() {
    uint32_t log_counter = 0;

    while (1) {
        // Wait for signal from periodic thread
        osEventFlagsWait(log_event, 0x1, osFlagsWaitAny, osWaitForever);

        log_counter++;

        // Get data from CAN frames
        int time = osKernelGetTickCount();
        
        int battery_soc = BMSFrame4::Instance().GetPackSOC();
        float battery_voltage = BMSFrame0::Instance().GetPackVoltage() / 10000.0f;
        float battery_current = BMSFrame2::Instance().GetPackCurrent() / 10.0f;
        int battery_avg_temp = BMSFrame1::Instance().GetAverageTemp();
        int battery_high_temp = BMSFrame1::Instance().GetHighTemp();

        int motor_rpm = MitsubaFrame0::Instance().GetMotorRPM();
        int motor_temp = MitsubaFrame0::Instance().GetFETTemp() * 5;

        float mppt1_input_voltage = MPPTInputMeasurementsFrame1::Instance().GetInputVoltage();
        float mppt1_input_current = MPPTInputMeasurementsFrame1::Instance().GetInputCurrent();
        float mppt2_input_voltage = MPPTInputMeasurementsFrame2::Instance().GetInputVoltage();
        float mppt2_input_current = MPPTInputMeasurementsFrame2::Instance().GetInputCurrent();
        float mppt3_input_voltage = MPPTInputMeasurementsFrame3::Instance().GetInputVoltage();
        float mppt3_input_current = MPPTInputMeasurementsFrame3::Instance().GetInputCurrent();

        int throttle = DriverControlsFrame0::Instance().GetThrottleVal() / 655;
        int regen = DriverControlsFrame0::Instance().GetRegenVal() / 655;
        int brake = DriverControlsFrame0::Instance().GetBrakeEnable();

        uint32_t bms_faults = BMSFrame4::Instance().GetFaultFlags0() |
                                (BMSFrame4::Instance().GetFaultFlags1() << 8) |
                                (BMSFrame4::Instance().GetFaultFlags2() << 16);
        uint32_t mitsuba_faults = MitsubaFrame2::Instance().GetAllFlags();

        // etl format specifier
        // (base, width, precision, upper_case, left_justified, boolalpha, showbase, fill)
        static constexpr etl::format_spec format_float(10, 8, 2, false, false, false, false, ' ');
        static constexpr etl::format_spec format_int(10, 6, 0, false, false, false, false, ' ');
        etl::string<8> float_buf;
        etl::string<6> int_buf;

        // Write data to line in file
        etl::to_string(time, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(battery_soc, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(battery_voltage, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(battery_current, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(battery_avg_temp, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(battery_high_temp, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(motor_rpm, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(motor_temp, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mppt1_input_voltage, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mppt1_input_current, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mppt2_input_voltage, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mppt2_input_current, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mppt3_input_voltage, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mppt3_input_current, float_buf, format_float, false);
        f_puts(float_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(throttle, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(regen, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(brake, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(bms_faults, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc(',', &fil);

        etl::to_string(mitsuba_faults, int_buf, format_int, false);
        f_puts(int_buf.c_str(), &fil);
        f_putc('\n', &fil);  // New line at the end of each data set

        // Sync the SD card every 10 log cycles (1s) to waste less time on f_sync()
        if (log_counter % 10) {
            if (f_sync(&fil) != FR_OK)
                Logger::LogError("Error syncing SD card\n");
        }
    }
}

/* Callback executed when IoTestFrame received
    This function is just for debugging CAN */
void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, IoTestFrame::GetOkLed());
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, IoTestFrame::GetErrorLed());
}

/* Callback executed when DriverControlsFrame0 received 
    Handle Mitsuba GPIO, throttle, and regen */
void DriverControls0Callback(uint8_t *data) {
    // If not in kill state or BMS trip, set motor state based on driver controls
    // TODO: For testing, do nothing here
    // if (!kill_state && !bms_trip)
    //     SetMotorState(DriverControlsFrame0::GetMotorEnable());
    // else
    //     SetMotorState(false);

    // Drive mode and direction based on driver controls
    SetMotorMode(DriverControlsFrame0::GetDriveMode());
    SetMotorDirection(DriverControlsFrame0::GetDriveDirection());

    // If regen or brake is active, set throttle to 0 and regen to value
    if (DriverControlsFrame0::GetRegenVal() > 0 || DriverControlsFrame0::GetBrakeEnable()) {
        SetThrottle(0);
        SetRegen(DriverControlsFrame0::GetRegenVal());
    }
    // If no regen or brake, set throttle to value and regen to 0
    else {
        SetThrottle(DriverControlsFrame0::GetThrottleVal());
        SetRegen(0);
    }

    // Set brake light
    if (DriverControlsFrame0::GetBrakeEnable()) {
        HAL_GPIO_WritePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RC_LIGHT_EN_GPIO_Port, RC_LIGHT_EN_Pin, GPIO_PIN_SET);
    } 

    // Set MPPT enable
    SetMPPTState(DriverControlsFrame0::GetSolarEnable());
}

/* Callback executed when DriverControlsFrame1 received
    Turn signals and hazards handled elsewhere, this callback is only for 
    headlights, push to talk, horn */
void DriverControls1Callback(uint8_t *data) {
    // Headlights
    HAL_GPIO_WritePin(HEADLIGHT_EN_GPIO_Port, 
                        HEADLIGHT_EN_Pin, 
                        static_cast<GPIO_PinState>(DriverControlsFrame1::GetHeadlight()));

    // Push to talk
    HAL_GPIO_WritePin(PTT_GPIO_Port, 
                        PTT_Pin,
                        static_cast<GPIO_PinState>(DriverControlsFrame1::GetPTT()));

    // Horn
    HAL_GPIO_WritePin(HORN_EN_GPIO_Port,
                        HORN_EN_Pin,
                        static_cast<GPIO_PinState>(DriverControlsFrame1::GetHorn()));
}

/* Callback executed when BMSFrame4 is received 
    Checks to see if any fault flags are set */
void BMSFaultCallback(uint8_t *data) {
    uint32_t fault_flags;
    memcpy(&fault_flags, BMSFrame4::Instance().Data(), 3);

    fault_flags &= ~(0x1 << 2);     // Ignore weak cell fault

    if (fault_flags) {
        Logger::LogError("BMS Fault: %lu\n", fault_flags);
        bms_trip = true;
        // Turn on strobe light
        HAL_GPIO_WritePin(STRB_LIGHT_EN_GPIO_Port, STRB_LIGHT_EN_Pin, GPIO_PIN_SET);

        // Disable motor
        SetMotorState(false);
    } else {
        bms_trip = false;
    }
}

/* Callback executed when any of the Mitsuba frames are received */
// TODO: Debug only, remove this in the future
void MitsubaCallback(uint8_t *data) {
    // Update the Mitsuba motor controller state
    // Logger::LogInfo("Mitsuba callback called\n");
}

/* Callback executed when kill switch is pressed */
// TODO: handle kill switch unpress logic
void KillSwitchCallback(void) {
    // Disable the motor
    SetMotorState(false);

    // Turn on strobe light
    HAL_GPIO_WritePin(STRB_LIGHT_EN_GPIO_Port, STRB_LIGHT_EN_Pin, GPIO_PIN_SET);

    // TODO: Store the kill status in EEPROM

    // Send kill switch status over CAN
    VCUFrame0::Instance().SetKillStatus(true);
    CANController::Send(&VCUFrame0::Instance());
}