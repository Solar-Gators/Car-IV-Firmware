#include "user.hpp"

#include "threads.h"


extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" SPI_HandleTypeDef hspi1;

/* FATFS globals */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;

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

/* Setup functions */
static void CAN_Modules_Init() {
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
}

bool SD_Init() {
    FILINFO fno;
    DIR dir;

    // Filename is in the format "LOGxxx.CSV"
    char log_filename[11];

    // Mount the filesystem
    fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        Logger::LogError("SD card mount failed\n");
        return false;
    } else {
        Logger::LogInfo("SD card mount successful\n");
    }

    // Open the root directory
    fres = f_opendir(&dir, "/");
    if (fres != FR_OK) {
        Logger::LogError("Failed to open root directory\n");
        return false;
    }

    // Find the next available log file number
    int log_num = 0;
    bool exists = true;
    while (exists) {
        sprintf(log_filename, "log%03d.csv", log_num);
        fres = f_stat(log_filename, &fno);
        if (fres != FR_OK) {
            exists = false;
        } else {
            log_num++;
            if (log_num > 999) {
                Logger::LogError("Too many log files");
                return false;
            }
        }
    }

    // Close the directory
    f_closedir(&dir);

    // Create a new file with log_filename
    fres = f_open(&fil, log_filename, FA_CREATE_NEW | FA_WRITE);
    if (fres != FR_OK) {
        Logger::LogError("Failed to create new file: %s", log_filename);
        return false;
    }

    // Write the csv header
    UINT bw;
    const char* csv_header = "time (ms),"\
                                "SoC,"\
                                "Battery Voltage,"\
                                "Battery Current,"\
                                "Battery Avg Temp,"\
                                "Battery High T,"\
                                "Motor RPM,"\
                                "Motor Temp,"\
                                "MPPT1 Voltage,"\
                                "MPPT1 Current,"\
                                "MPPT2 Voltage,"\
                                "MPPT2 Current,"\
                                "MPPT3 Voltage,"\
                                "MPPT3 Current,"\
                                "Throttle,"\
                                "Regen,"\
                                "Brake,"\
                                "BMS Faults,"\
                                "MC Faults\n";
    f_write(&fil, csv_header, strlen(csv_header), &bw);
    if (fres != FR_OK || bw < strlen(csv_header)) {
        Logger::LogError("Failed to write new log file");
        return false;
    }

    // Flush file
    f_sync(&fil);

    Logger::LogInfo("Created new file: %s", log_filename);

    return true;
}

/* User entry function, called from main before kernel initialization */
void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Initialize CAN things
    CAN_Modules_Init();

    // Setup kill switch callbacks
    // TODO: Check that kill switch is not pressed on startup
    kill_sw.RegisterNormalPressCallback(KillSwitchCallback);

    // Initialize SD card
    // TODO: Do not setup SD thread if SD_Init() fails
    SD_Init();

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