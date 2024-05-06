#include "user.hpp"
#include <string>
#include "IoTestFrame.hpp"
#include "MotorControlFrame.hpp"
#include "threads.h"
#include "TMC4671.hpp"

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
extern "C" SPI_HandleTypeDef hspi1;

extern void ThreadsStart();

/* Initialize CAN frames and devices */

CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

extern "C" void CPP_UserSetup(void);

TMC4671 FOC = TMC4671(&hspi1, GPIOC, GPIO_PIN_4); 

void CAN_Modules_Init() {
    // Add CAN devices and CAN frames
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);
    CANController::AddFilterAll();
    CANController::Start();
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

   

    ThreadsStart();    
}

