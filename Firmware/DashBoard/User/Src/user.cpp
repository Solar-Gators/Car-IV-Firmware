#include "user.hpp"
#include <string>
#include "IoTestFrame.hpp"
#include "MotorControlFrame.hpp"
#include "ADS7138.hpp"
#include "threads.h"

using namespace std;

/* In this program, two tasks are generated. One is triggered by a periodic timer, which
 * expires every 500ms. The other is a regular thread, which is triggered by and event flag.
 * The periodic timer task sets the event flag, which causes the regular thread to exit
 * its wait state and execute. The regular thread then toggles the LED and prints a message.
 * 
 * CPP_UserSetup is called from main.cpp, and is where the user should put their setup code.
 * It is run before the RTOS scheduler is started.
*/
extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" I2C_HandleTypeDef hi2c2;

extern void ThreadsStart();

/* Initialize CAN frames and devices */

CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);


ADS7138 adcs[1] = {ADS7138(&hi2c2, 0x10)};


extern "C" void CPP_UserSetup(void);






void CAN_Modules_Init() {
    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddFilterAll();
    CANController::Start();
}
void ADC_Modules_Init() {
   
    
    if (adcs[0].Init() != HAL_OK || adcs[0].TestI2C() != HAL_OK)
        Logger::LogError("ADC %d init failed", 0);
    else
        Logger::LogInfo("ADC %d init success", 0);

    if (adcs[0].ConfigureOversampling(OsrCfg_Type::OSR_16) != HAL_OK)
        Logger::LogError("ADC %d configure oversampling failed", 0);

    // Set all ADCs to initiate conversion on request
    if (adcs[0].ConfigureOpmode(false, ConvMode_Type::MANUAL) != HAL_OK)
        Logger::LogError("ADC %d configure opmode failed", 0);

    // For all ADCs, append channel ID to data
    if (adcs[0].ConfigureData(false, DataCfg_AppendType::ID) != HAL_OK)
        Logger::LogError("ADC %d configure data failed", 0);

    // Set oversampling rate to 16
    if (adcs[0].ConfigureOversampling(OsrCfg_Type::OSR_16) != HAL_OK)
        Logger::LogError("ADC %d configure oversampling failed", 0);


    // For adc0, sequence channels 5, 7 for current sense
    if (adcs[0].AutoSelectChannels((0x1 << 0)) != HAL_OK)
       Logger::LogError("ADC 0 auto select channels failed");

    
}

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);
    
    // CAN_Modules_Init();


    

    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    //CANController::AddRxMessage(&io_test_frame, IoMsgCallback);
    CANController::AddFilterAll();
    CANController::Start();

    ADC_Modules_Init();

    ThreadsStart();    
}

uint16_t readThrottleValue() {
    //heart beat
    //Logger::LogInfo("Periodic timer fired\n");
   
   
    uint16_t rawData;
    //IMU_ReadAccel(&imu);
    adcs[0].ConversionReadAutoSequence(&rawData, 1);

    return(rawData);


}
