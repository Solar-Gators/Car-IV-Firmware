#include "user.hpp"

#include "threads.h"


extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" SPI_HandleTypeDef hspi1;

/* FATFS globals */
FATFS fs;
FATFS *pfs;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;

/* Initialize CAN frames and devices */
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

/* Initialize DACs for throttle and regen */
DACx311 throttle_dac = DACx311(&hspi1, THROTTLE_CS_GPIO_Port, THROTTLE_CS_Pin);
DACx311 regen_dac = DACx311(&hspi1, REGEN_CS_GPIO_Port, REGEN_CS_Pin);

/* Initialize kill switch button */
/* Debounce = 50ms, default state = 1 */
// TODO: Use simple debounce instead? Need to register press and unpress
Button kill_sw = Button(KILL_SW_GPIO_Port, KILL_SW_Pin, 50, GPIO_PIN_SET);

/* Setup function, called from main before kernel initialization */
void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddRxMessage(&IoTestFrame::Instance(), IoMsgCallback);
    CANController::AddRxMessage(&DriverControlsFrame0::Instance(), DriverControls0Callback);
    CANController::AddRxMessage(&DriverControlsFrame1::Instance(), DriverControls1Callback);
    CANController::AddRxMessage(&MitsubaFrame0::Instance(), MitsubaCallback);
    CANController::AddRxMessage(&MitsubaFrame1::Instance(), MitsubaCallback);
    CANController::AddRxMessage(&MitsubaFrame2::Instance(), MitsubaCallback);
    CANController::AddFilterAll();
    CANController::Start();

    // Permanently request all Mitsuba frames
    MitsubaRequestFrame::Instance().SetRequestAll();

    // Setup kill switch callbacks
    // TODO: Check that kill switch is not pressed on startup
    kill_sw.RegisterNormalPressCallback(KillSwitchCallback);

    // Attempt to mount SD card
    if (f_mount(&fs, "", 1) != FR_OK) {
        Logger::LogError("SD card mount failed\n");
    } else {
        Logger::LogInfo("SD card mount successful\n");
    }

    // Turn on headlights
    HAL_GPIO_WritePin(HEADLIGHT_EN_GPIO_Port, HEADLIGHT_EN_Pin, GPIO_PIN_SET);

    // TODO: Eventually get rid of this, get state from driver controls
    SetThrottle(0);
    SetRegen(0);
    // Enable the motor
    SetMotorState(true);
    // Set the motor mode to eco
    SetMotorMode(true);
    // Set the motor direction to forward
    SetMotorDirection(false);

    // Start periodic tasks
    ThreadsStart();
}

/* Helper Functions */
void SetMotorState(bool state) {
    HAL_GPIO_WritePin(MC_MAIN_CTRL_GPIO_Port, MC_MAIN_CTRL_Pin, static_cast<GPIO_PinState>(state));
}

void SetMotorMode(bool mode) {
    HAL_GPIO_WritePin(MC_PE_CTRL_GPIO_Port, MC_PE_CTRL_Pin, static_cast<GPIO_PinState>(mode));
}

void SetMotorDirection(bool direction) {
    HAL_GPIO_WritePin(MC_FR_CTRL_GPIO_Port, MC_FR_CTRL_Pin, static_cast<GPIO_PinState>(direction));
}

void SetThrottle(uint16_t value) {
    throttle_dac.SetValue(value);
}

void SetRegen(uint16_t value) {
    regen_dac.SetValue(value);
}