#include "threads.h"

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

/* Start periodic threads */
void ThreadsStart() {
    // Request to receive Mitsuba frames every 500ms
    osTimerStart(mitsuba_req_periodic_timer_id, 500);

    // Toggle lights (hazards & turn signals) every 500ms
    osTimerStart(toggle_lights_periodic_timer_id, 500);
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
    // Hazards toggle the RC light no matter what
    if (DriverControlsFrame1::GetHazards()) {
        HAL_GPIO_TogglePin(RC_LIGHT_EN_GPIO_Port, RC_LIGHT_EN_Pin);
        // If brake is not enabled, hazards also toggle all side lights
        if (!DriverControlsFrame0::GetBrakeEnable()) {
            HAL_GPIO_TogglePin(FL_LIGHT_EN_GPIO_Port, FL_LIGHT_EN_Pin);
            HAL_GPIO_TogglePin(FR_LIGHT_EN_GPIO_Port, FR_LIGHT_EN_Pin);
            HAL_GPIO_TogglePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin);
            HAL_GPIO_TogglePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin);
        }
    }

    // Turn signals toggle side lights no matter what
    // This implies that hazards will override turn signals
    else if (DriverControlsFrame1::GetLeftTurn()) {
        HAL_GPIO_TogglePin(FL_LIGHT_EN_GPIO_Port, FL_LIGHT_EN_Pin);
        HAL_GPIO_TogglePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin);
    }
    else if (DriverControlsFrame1::GetRightTurn()) {
        HAL_GPIO_TogglePin(FR_LIGHT_EN_GPIO_Port, FR_LIGHT_EN_Pin);
        HAL_GPIO_TogglePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin);
    }

    // If no turn signals, hazards, or brake, turn off all lights
    else if (!DriverControlsFrame0::GetBrakeEnable()) {
        HAL_GPIO_WritePin(FL_LIGHT_EN_GPIO_Port, FL_LIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FR_LIGHT_EN_GPIO_Port, FR_LIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RL_LIGHT_EN_GPIO_Port, RL_LIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RR_LIGHT_EN_GPIO_Port, RR_LIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RC_LIGHT_EN_GPIO_Port, RC_LIGHT_EN_Pin, GPIO_PIN_RESET);
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
    // On/off, mode, and direction based solely on driver controls
    // TODO: Error states should turn off motor
    SetMotorState(DriverControlsFrame0::GetMotorEnable());
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
}

/* Callback executed when DriverControlsFrame1 received
    Turn signals and hazards handled elsewhere, this callback is only for 
    headlights, push to talk, horn */
void DriverControls1Callback(uint8_t *data) {
    // Headlights
    HAL_GPIO_WritePin(FL_LIGHT_EN_GPIO_Port, 
                        FL_LIGHT_EN_Pin, 
                        static_cast<GPIO_PinState>(DriverControlsFrame1::GetHeadlight()));
    HAL_GPIO_WritePin(FR_LIGHT_EN_GPIO_Port, 
                        FR_LIGHT_EN_Pin, 
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