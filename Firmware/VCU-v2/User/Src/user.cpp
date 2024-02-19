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

/* Initialize DACs */
DACx311 throttle_dac = DACx311(&hspi1, THROTTLE_CS_GPIO_Port, THROTTLE_CS_Pin);
DACx311 regen_dac = DACx311(&hspi1, REGEN_CS_GPIO_Port, REGEN_CS_Pin);


void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddRxMessage(&mitsuba_frame_1);
    CANController::AddFilterAll();
    CANController::Start();

    uint8_t dac_value = 50;
    while (1) {
        throttle_dac.SetValue(dac_value << 8);
        dac_value += 50;
    }

    throttle_dac.SetValue(0x0FFF);
}