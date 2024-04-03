#include "user.hpp"

#include "threads.h"

#include "etl/format_spec.h"
#include "etl/to_string.h"

extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" SPI_HandleTypeDef hspi1;
extern "C" I2C_HandleTypeDef hi2c2;

/* FATFS globals */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;

/* Global states */
bool kill_state = true;
bool bms_trip = false;

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
static void Default_Outputs() {
    // Turn off motor
    // TODO: For testing only, start motor on
    SetMotorState(true);

    // Set throttle and regen to 0
    SetThrottle(0x0);
    SetRegen(0x0);

    // Turn off MPPT contactors
    SetMPPTState(false);

    // Turn on headlights
    HAL_GPIO_WritePin(HEADLIGHT_EN_GPIO_Port, HEADLIGHT_EN_Pin, GPIO_PIN_SET);
}

static void CAN_Modules_Init() {
    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);

    // Add custom frames
    CANController::AddRxMessage(&IoTestFrame::Instance(), IoMsgCallback);
    CANController::AddRxMessage(&DriverControlsFrame0::Instance(), DriverControls0Callback);
    CANController::AddRxMessage(&DriverControlsFrame1::Instance(), DriverControls1Callback);

    // Add BMS frames
    // BMSFrame4 contains fault flags
    CANController::AddRxMessage(&BMSFrame0::Instance());
    CANController::AddRxMessage(&BMSFrame1::Instance());
    CANController::AddRxMessage(&BMSFrame2::Instance());
    CANController::AddRxMessage(&BMSFrame3::Instance());
    CANController::AddRxMessage(&BMSFrame4::Instance());
    CANController::AddRxMessage(&BMSFrame5::Instance());

    // Add motor controller frames
    CANController::AddRxMessage(&MitsubaFrame0::Instance(), MitsubaCallback);
    CANController::AddRxMessage(&MitsubaFrame1::Instance(), MitsubaCallback);
    CANController::AddRxMessage(&MitsubaFrame2::Instance(), MitsubaCallback);

    // Add MPPT frames, no callbacks associated with these
    // Don't care about MPPT outputs or preset limits
    CANController::AddRxMessage(&MPPTInputMeasurementsFrame1::Instance());
    // CANController::AddRxMessage(&MPPTOutputMeasurementsFrame1::Instance());
    CANController::AddRxMessage(&MPPTTemperatureFrame1::Instance());
    CANController::AddRxMessage(&MPPTAuxPowerFrame1::Instance());
    // CANController::AddRxMessage(&MPPTLimitsFrame1::Instance());
    CANController::AddRxMessage(&MPPTInputMeasurementsFrame2::Instance());
    // CANController::AddRxMessage(&MPPTOutputMeasurementsFrame2::Instance());
    CANController::AddRxMessage(&MPPTTemperatureFrame2::Instance());
    CANController::AddRxMessage(&MPPTAuxPowerFrame2::Instance());
    // CANController::AddRxMessage(&MPPTLimitsFrame2::Instance());
    CANController::AddRxMessage(&MPPTInputMeasurementsFrame3::Instance());
    // CANController::AddRxMessage(&MPPTOutputMeasurementsFrame3::Instance());
    CANController::AddRxMessage(&MPPTTemperatureFrame3::Instance());
    CANController::AddRxMessage(&MPPTAuxPowerFrame3::Instance());
    // CANController::AddRxMessage(&MPPTLimitsFrame3::Instance());


    // Accept all messages
    CANController::AddFilterAll();
    CANController::Start();

    // Permanently request all Mitsuba frames
    MitsubaRequestFrame::Instance().SetRequestAll();
}

bool SD_Init() {
    FILINFO fno;
    DIR dir;

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

    // Find the next available log file number using binary search
    int low = 0;
    int high = 999;
    int middle;
    bool found = false;

    static constexpr etl::format_spec format_int(10, 3, 0, false, false, false, false, '0');
    etl::string<11> log_filename = "log000.csv";
    etl::string<3> log_number_string;

    while (low <= high) {
        middle = low + (high-low) / 2;

        // Update the number part of the log_filename
        etl::to_string(middle, log_number_string, format_int, false);
        std::copy(log_number_string.begin(), log_number_string.end(), log_filename.begin() + 3);

        // Search for file
        fres = f_stat(log_filename.c_str(), &fno);
        if (fres == FR_OK) {
            // File exists, search in higher half
            low = middle + 1;
        } else {
            // File does not exist, check if the previous file exists
            etl::to_string(middle-1, log_number_string, format_int, false);
            std::copy(log_number_string.begin(), log_number_string.end(), log_filename.begin() + 3);
            fres = f_stat(log_filename.c_str(), &fno);

            // If file not found, search in lower half
            if (fres != FR_OK) {
                high = middle - 1;
            }

            // If file is found, current middle is the next available file
            else {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        Logger::LogError("Failed to find available log file number");
        return false;
    }

    // middle holds the next available log file number
    etl::to_string(middle, log_number_string, format_int, false);
    std::copy(log_number_string.begin(), log_number_string.end(), log_filename.begin() + 3);

    // Close the directory
    f_closedir(&dir);

    // Create a new file with log_filename
    fres = f_open(&fil, log_filename.c_str(), FA_CREATE_NEW | FA_WRITE);
    if (fres != FR_OK) {
        Logger::LogError("Failed to create new file: %s", log_filename.c_str());
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

    Logger::LogInfo("Created new file: %s", log_filename.c_str());

    return true;
}

/* User entry function, called from main before kernel initialization */
void CPP_UserSetup(void) {
    // Set initial GPIO states
    Default_Outputs();

    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Initialize CAN things
    CAN_Modules_Init();

    // Read initial kill switch state
    if (kill_sw.ReadPin() == GPIO_PIN_SET)
        kill_state = false;

    // Setup kill switch callbacks
    kill_sw.RegisterNormalPressCallback(KillSwitchCallback);

    // Initialize SD card
    // TODO: Do not setup SD thread if SD_Init() fails
    SD_Init();

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

void SetMPPTState(bool state) {
    // If turning MPPTs off
    if (!state) {
        HAL_GPIO_WritePin(MPPT_CONTACTOR_EN_GPIO_Port, MPPT_CONTACTOR_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MPPT_PRECHARGE_EN_GPIO_Port, MPPT_PRECHARGE_EN_Pin, GPIO_PIN_RESET);
    }
    // If turning MPPTs on
    else {
        // If MPPTs are already on, do nothing
        if (HAL_GPIO_ReadPin(MPPT_CONTACTOR_EN_GPIO_Port, MPPT_CONTACTOR_EN_Pin) == GPIO_PIN_SET) {
            return;
        }

        // Turn on MPPT contactors
        HAL_GPIO_WritePin(MPPT_PRECHARGE_EN_GPIO_Port, MPPT_PRECHARGE_EN_Pin, GPIO_PIN_SET);
        osDelay(400);
        HAL_GPIO_WritePin(MPPT_CONTACTOR_EN_GPIO_Port, MPPT_CONTACTOR_EN_Pin, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(MPPT_PRECHARGE_EN_GPIO_Port, MPPT_PRECHARGE_EN_Pin, GPIO_PIN_RESET);
    }
}

void SetThrottle(uint16_t value) {
    throttle_dac.SetValue(value);
}

void SetRegen(uint16_t value) {
    regen_dac.SetValue(value);
}