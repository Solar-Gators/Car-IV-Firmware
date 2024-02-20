#include "user.hpp"

/* In this program, two tasks are generated. One is triggered by a periodic timer, which
 * expires every 500ms. The other is a regular thread, which is triggered by and event flag.
 * The periodic timer task sets the event flag, which causes the regular thread to exit
 * its wait state and execute. The regular thread then toggles the LED and prints a message.
 * 
 * CPP_UserSetup is called from main.cpp, and is where the user should put their setup code.
 * It is run before the RTOS scheduler is started.
*/

extern "C" void CPP_UserSetup(void);

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;
extern "C" SPI_HandleTypeDef hspi1;

/* Initialize CAN frames and devices */
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

/* Initialize Other Hardware */
DACx311 throttle_dac = DACx311(&hspi1, THROTTLE_CS_GPIO_Port, THROTTLE_CS_Pin);
DACx311 regen_dac = DACx311(&hspi1, REGEN_CS_GPIO_Port, REGEN_CS_Pin);
Mitsuba mitsuba = Mitsuba(&throttle_dac, 
                        &regen_dac, 
                        MC_MAIN_CTRL_GPIO_Port, 
                        MC_MAIN_CTRL_Pin,
                        MC_PE_CTRL_GPIO_Port,
                        MC_PE_CTRL_Pin,
                        MC_FR_CTRL_GPIO_Port,
                        MC_FR_CTRL_Pin);

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddRxMessage(&mitsuba_frame_1);
    CANController::AddFilterAll();
    CANController::Start();

    mitsuba.SetThrottle(0);
    mitsuba.Enable();

    uint8_t throttle = 0;
    while (1) {
        mitsuba.SetThrottle(throttle << 8);
        throttle += 3;
        HAL_Delay(250);
    }
}