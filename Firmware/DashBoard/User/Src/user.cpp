#include "user.hpp"
#include "IMU.h"
// #include "usbd_cdc_if.h"
#include <string>
#include "IoTestFrame.hpp"
#include "MotorControlFrame.hpp"
#include "ADS7138.hpp"

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

/* Initialize CAN frames and devices */

CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);


ADS7138 adcs[1] = {ADS7138(&hi2c2, 0x10)};


extern "C" void CPP_UserSetup(void);

/* Task function prototypes */
void PeriodicTask1(void *argument);

/* Periodic timer definitions */
osTimerAttr_t periodic_timer_attr = {
    .name = "Periodic Task 1",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_timer_id = osTimerNew((osThreadFunc_t)PeriodicTask1, osTimerPeriodic, NULL, &periodic_timer_attr);

uint16_t rawData;
uint8_t high;
uint8_t low;
uint8_t ignitionState = false;
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

    // Set all ADCs to initiate conversion on request
    if (adcs[0].ConfigureOpmode(false, ConvMode_Type::MANUAL) != HAL_OK)
        Logger::LogError("ADC %d configure opmode failed", 0);

    // For all ADCs, append channel ID to data
    if (adcs[0].ConfigureData(false, DataCfg_AppendType::ID) != HAL_OK)
        Logger::LogError("ADC %d configure data failed", 0);

    if (adcs[0].AutoSelectChannels((0x1 << 0)) != HAL_OK)
       Logger::LogError("ADC 0 auto select channels failed");
    
}

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);
    
    CAN_Modules_Init();

    ADC_Modules_Init();

    // 20Hz periodic timer
    osTimerStart(periodic_timer_id, 50);
}

static uint8_t counter = 0;
void PeriodicTask1(void *argument) {
    // Read brakes
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET){
       DriverControlsFrame0::SetBrake(true);
    }else{
        DriverControlsFrame0::SetBrake(false);
    }

    // Read throttle
    adcs[0].ConversionReadAutoSequence(&rawData, 1);
    DriverControlsFrame0::SetThrottleVal(rawData << 4);
    
    // Read shutdown status
    if(HAL_GPIO_ReadPin(GPIOC, 6) == GPIO_PIN_RESET){
       DriverControlsFrame0::SetShutdownStatus((true));
    }

    CANController::Send(&DriverControlsFrame0::Instance());

    //heart beat
    if (counter++ % 10 == 0)
        HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
}
