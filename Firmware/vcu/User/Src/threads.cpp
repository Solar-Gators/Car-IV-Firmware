#include "threads.h"

#include <algorithm>

#include "etl/to_string.h"
#include "etl/format_spec.h"
#include "etl/string_utilities.h"

/* Setup data structures */
osEventFlagsId_t log_event = osEventFlagsNew(NULL);
osEventFlagsId_t strobe_event = osEventFlagsNew(NULL);
StaticSemaphore_t throttle_mutex_cb;
const osMutexAttr_t throttle_mutex_attr = {
    .name = "Throttle Mutex",
    .attr_bits = osMutexRecursive | osMutexPrioInherit | osMutexRobust,
    .cb_mem = &throttle_mutex_cb,
    .cb_size = sizeof(throttle_mutex_cb),
};
osMutexId_t throttle_mutex_id = osMutexNew(NULL);

/* Setup periodic threads */
osTimerAttr_t mitsuba_req_periodic_timer_attr = {
    .name = "Send Mitsuba Request Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t mitsuba_req_periodic_timer_id = osTimerNew(
                                            (osThreadFunc_t)SendMitsubaRequest, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &mitsuba_req_periodic_timer_attr);

osTimerAttr_t toggle_lights_periodic_timer_attr = {
    .name = "Toggle Lights Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t toggle_lights_periodic_timer_id = osTimerNew(
                                            (osThreadFunc_t)ToggleLights, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &toggle_lights_periodic_timer_attr);

osTimerAttr_t logger_periodic_timer_attr = {
    .name = "SD Logger Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t logger_periodic_timer_id = osTimerNew(
                                            (osThreadFunc_t)LogDataPeriodic, 
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
osThreadId_t logger_thread_id = osThreadNew(
                                        (osThreadFunc_t)LogData, 
                                        NULL, 
                                        &logger_thread_attributes);

uint32_t strobe_thread_buffer[2048];
StaticTask_t strobe_thread_control_block;
const osThreadAttr_t strobe_thread_attributes = {
    .name = "Strobe Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &strobe_thread_control_block,
    .cb_size = sizeof(strobe_thread_control_block),
    .stack_mem = &strobe_thread_buffer[0],
    .stack_size = sizeof(strobe_thread_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t strobe_thread_id = osThreadNew(
                                        (osThreadFunc_t)StrobeThread, 
                                        NULL, 
                                        &strobe_thread_attributes);

/* Setup timers */
osTimerId_t throttle_timer = osTimerNew(
                                    (osThreadFunc_t)ThrottleTimeoutCallback, 
                                    osTimerOnce, 
                                    NULL,
                                    NULL);

/* Start periodic threads */
void ThreadsStart() {
    // Request to receive Mitsuba frames every 500ms
    // This thread is also used for throttle watchdog
    osTimerStart(mitsuba_req_periodic_timer_id, 500);

    // Toggle lights (hazards & turn signals) every 500ms
    osTimerStart(toggle_lights_periodic_timer_id, 500);

    // Toggle logger thread every 100ms if SD card is present
    if (sd_present)
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
    if (DriverControlsFrame1::GetHazards() || 
        DriverControlsFrame1::GetLeftTurn()) {
        HAL_GPIO_TogglePin(FL_LIGHT_EN_GPIO_Port, FL_LIGHT_EN_Pin);
        HAL_GPIO_TogglePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin);
    }
    // If neither hazards nor left turn on, left lights depend on brake lights
    else {
        HAL_GPIO_WritePin(FL_LIGHT_EN_GPIO_Port, 
                          FL_LIGHT_EN_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin, 
                            static_cast<GPIO_PinState>(
                                    DriverControlsFrame0::GetBrake()));
    }
    // If hazards or right turn on, toggle right lights
    if (DriverControlsFrame1::GetHazards() || 
        DriverControlsFrame1::GetRightTurn()) {
        HAL_GPIO_TogglePin(FR_LIGHT_EN_GPIO_Port, FR_LIGHT_EN_Pin);
        HAL_GPIO_TogglePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin);
    }
    // If neither hazards nor right turn on, right lights depend on brake lights
    else {
        HAL_GPIO_WritePin(FR_LIGHT_EN_GPIO_Port, 
                          FR_LIGHT_EN_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin, 
                            static_cast<GPIO_PinState>(
                                    DriverControlsFrame0::GetBrake()));
    }

    // If brake lights are not on, center light turns off
    if (!DriverControlsFrame0::GetBrake()) {
        HAL_GPIO_WritePin(RC_LIGHT_EN_GPIO_Port, 
                          RC_LIGHT_EN_Pin, 
                          GPIO_PIN_RESET);
    }

    // If in kill state or BMS trip, turn on strobe light, otherwise off
    if (kill_state || bms_trip)
        osEventFlagsSet(strobe_event, 0x1);
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
        
        int battery_soc = BMSFrame3::Instance().GetPackSoC();
        float battery_voltage = BMSFrame0::Instance().GetPackVoltage() / 
                                10000.0f;
        float battery_current = BMSFrame1::Instance().GetPackCurrent() / 
                                10.0f;
        int battery_low_temp = BMSFrame2::Instance().GetLowTemp();
        int battery_high_temp = BMSFrame2::Instance().GetHighTemp();

        int motor_rpm = MitsubaFrame0::Instance().GetMotorRPM();
        int motor_temp = MitsubaFrame0::Instance().GetFETTemp() * 5;

        float mppt1_input_voltage = MPPTInputMeasurementsFrame1::Instance().
                                                            GetInputVoltage();
        float mppt1_input_current = MPPTInputMeasurementsFrame1::Instance().
                                                            GetInputCurrent();
        float mppt2_input_voltage = MPPTInputMeasurementsFrame2::Instance().
                                                            GetInputVoltage();
        float mppt2_input_current = MPPTInputMeasurementsFrame2::Instance().
                                                            GetInputCurrent();
        float mppt3_input_voltage = MPPTInputMeasurementsFrame3::Instance().
                                                            GetInputVoltage();
        float mppt3_input_current = MPPTInputMeasurementsFrame3::Instance().
                                                            GetInputCurrent();

        int throttle = DriverControlsFrame0::Instance().GetThrottleVal() / 
                                                        655;
        int regen = static_cast<int>(DriverControlsFrame1::Instance().
                                                        GetRegenEnable());
        int brake = static_cast<int>(DriverControlsFrame0::Instance().
                                                        GetBrake());

        uint32_t bms_faults = BMSFrame3::Instance().GetFaultFlags();
        uint32_t mitsuba_faults = MitsubaFrame2::Instance().GetAllFlags();

        // etl format specifier
        // (base, width, precision, upper_case, 
        //  left_justified, boolalpha, showbase, fill)
        static constexpr etl::format_spec format_float(10, 8, 2, 
                                                    false, false, false, 
                                                    false, ' ');
        static constexpr etl::format_spec format_int(10, 6, 0, 
                                                    false, false, false, 
                                                    false, ' ');
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

        etl::to_string(battery_low_temp, int_buf, format_int, false);
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

        // Sync the SD card every 10 log cycles (1s) 
        // to waste less time on f_sync()
        if (log_counter % 10) {
            if (f_sync(&fil) != FR_OK)
                Logger::LogError("Error syncing SD card\n");
        }
    }
}

/* Thread runs when strobe light is turned on */
void StrobeThread() {
    while (1) {
        if (kill_state || bms_trip) {
            for (int i = 0; i < 6; i++) {
                HAL_GPIO_TogglePin(STRB_LIGHT_EN_GPIO_Port, STRB_LIGHT_EN_Pin);
                osDelay(100);
            }
            for (int i = 0; i < 6; i++) {
                HAL_GPIO_TogglePin(STRB_LIGHT_EN_GPIO_Port, STRB_LIGHT_EN_Pin);
                osDelay(200);
            }
            if (kill_sw.ReadPin() == GPIO_PIN_SET)
                KillSwitchCallback();
        } else {
            HAL_GPIO_WritePin(STRB_LIGHT_EN_GPIO_Port, 
                              STRB_LIGHT_EN_Pin,
                              GPIO_PIN_RESET);
            osEventFlagsWait(strobe_event, 0x1, osFlagsWaitAny, osWaitForever);
        }
    }
}

/* Callback executed when IoTestFrame received
    This function is just for debugging CAN */
void IoMsgCallback(uint8_t *data) {
    // Set LEDs based on info in message
    HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, 
                      IoTestFrame::GetOkLed());
    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 
                      IoTestFrame::GetErrorLed());
}

/* Callback executed when DriverControlsFrame0 received 
    Handle Mitsuba GPIO, throttle, and regen */
void DriverControls0Callback(uint8_t *data) {
    // If shutdown is asserted, sync filesystem, send kill status
    if (DriverControlsFrame0::GetShutdownStatus()) {
        if (f_sync(&fil) != FR_OK)
            Logger::LogError("Error syncing SD card\n");
        VCUFrame0::Instance().SetKillStatus(true);
        CANController::Send(&VCUFrame0::Instance());
    }

    // If not in kill state or BMS trip, 
    // set motor state based on driver controls
    if (!kill_state && !bms_trip)
        SetMotorState(DriverControlsFrame1::GetMotorEnable());
    else
        SetMotorState(false);

    // Drive mode and direction based on driver controls
    SetMotorDirection(DriverControlsFrame1::GetDriveDirection());
    SetMotorMode(DriverControlsFrame1::GetDriveMode());

    // If regen or brake is active, set throttle to 0 and regen to value
    if (DriverControlsFrame1::GetRegenEnable() || 
        DriverControlsFrame0::GetBrake()) {
        osMutexAcquire(throttle_mutex_id, osWaitForever);
        SetThrottle(0);
        if (DriverControlsFrame1::GetRegenEnable())
            // TODO: Calculate max regen value
            SetRegen(10000);
        osMutexRelease(throttle_mutex_id);
    }
    // If no regen or brake, set throttle to value and regen to 0
    else {
        uint16_t throttle = 0;
        if (DriverControlsFrame0::GetThrottleVal() < 58000)
            throttle = DriverControlsFrame0::GetThrottleVal();
        else
            throttle = 58000;
        osMutexAcquire(throttle_mutex_id, osWaitForever);
        SetThrottle(throttle);
        SetRegen(0);
        osMutexRelease(throttle_mutex_id);
    }

    // Start throttle timeout
    osTimerStart(throttle_timer, 250);

    // Set brake light
    if (DriverControlsFrame0::GetBrake()) {
        HAL_GPIO_WritePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RC_LIGHT_EN_GPIO_Port, RC_LIGHT_EN_Pin, GPIO_PIN_SET);
    } 

    // Set MPPT enable
    SetMPPTState(DriverControlsFrame1::GetPVEnable());
}

/* Callback executed when DriverControlsFrame1 received
    Turn signals and hazards handled elsewhere, this callback is only for 
    headlights, push to talk, horn */
void DriverControls1Callback(uint8_t *data) {
    // Headlights
    HAL_GPIO_WritePin(HEADLIGHT_EN_GPIO_Port, 
                      HEADLIGHT_EN_Pin, 
                      static_cast<GPIO_PinState>(
                      DriverControlsFrame1::GetHeadlight()));

    // Push to talk
    HAL_GPIO_WritePin(PTT_GPIO_Port, 
                      PTT_Pin,
                      static_cast<GPIO_PinState>(
                      DriverControlsFrame1::GetPTT()));

    // Horn
    HAL_GPIO_WritePin(HORN_EN_GPIO_Port,
                      HORN_EN_Pin,
                      static_cast<GPIO_PinState>(
                      DriverControlsFrame1::GetHorn()));

    // Motor mode and direction
    // Motor enable handled in DriverControlsFrame0 callback
    SetMotorMode(DriverControlsFrame1::GetDriveMode());
    SetMotorDirection(DriverControlsFrame1::GetDriveDirection());
}

/* Callback executed when BMSFrame3 is received 
    Checks to see if any fault flags are set */
void BMSFaultCallback(uint8_t *data) {
    uint32_t fault_flags = BMSFrame3::Instance().GetFaultFlags();

    fault_flags &= ~(0x1 << 7);     // Ignore kill switch fault, generated here

    if (fault_flags) {
        Logger::LogError("BMS Fault: %lu\n", fault_flags);
        bms_trip = true;
        // Turn on strobe light
        osEventFlagsSet(strobe_event, 0x1);

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
    Logger::LogInfo("Mitsuba callback called\n");
}

/* Callback executed when kill switch is pressed */
void KillSwitchCallback(void) {
    if (kill_sw.ReadPin() == GPIO_PIN_SET) {
        // If kill switch is pressed, set kill state to true
        kill_state = true;
        Logger::LogError("Kill switch pressed");

        // Turn on strobe light
        osEventFlagsSet(strobe_event, 0x1);

        // Turn on error LED
        HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);

        // Send kill switch status over CAN
        VCUFrame0::Instance().SetKillStatus(true);
        CANController::Send(&VCUFrame0::Instance());
    } else {
        // If kill switch is unpressed, set kill state to false
        kill_state = false;
        Logger::LogInfo("Kill switch unpressed");

        // Turn off strobe light
        HAL_GPIO_WritePin(STRB_LIGHT_EN_GPIO_Port, 
                          STRB_LIGHT_EN_Pin, 
                          GPIO_PIN_RESET);

        // Turn off error LED
        HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);

        // Send kill switch status over CAN
        VCUFrame0::Instance().SetKillStatus(false);
        CANController::Send(&VCUFrame0::Instance());
    }
}

/* Callback executed when throttle update has not been received */
void ThrottleTimeoutCallback(void) {
    Logger::LogError("Throttle timeout");
    // If throttle update has not been received in 100ms, set throttle to 0
    osMutexAcquire(throttle_mutex_id, osWaitForever);
    SetThrottle(0);
    osMutexRelease(throttle_mutex_id);
}