#include "threads.h"

#include "MotorControlFrame.hpp"
#include "ADS7138.hpp"
#include "user.hpp"

/* Periodic timer definitions */
osTimerAttr_t throttle_timer_attr = {
    .name = "Periodic Task 1",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t throttle_timer_id = osTimerNew((osThreadFunc_t)readThrottle, osTimerPeriodic, NULL, &throttle_timer_attr);

osTimerAttr_t brake_timer_attr = {
    .name = "breakMonitor",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t brake_timer_id = osTimerNew((osThreadFunc_t)readBrakes, osTimerPeriodic, NULL, &brake_timer_attr);


/* Start periodic threads */
void ThreadsStart() {

    osTimerStart(throttle_timer_id, 50);
    osTimerStart(brake_timer_id, 20);
}


void readThrottle() {
    //heart beat
    //Logger::LogInfo("Periodic timer fired\n");
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

   
    
   
 
    

    
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == false){
       DriverControlsFrame0::SetShutdownStatus((true));
    }
    
    CANController::Send(&DriverControlsFrame0::Instance());


}

void readBrakes(){

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == true){
        DriverControlsFrame0::SetBrake((true));
        //DriverControlsFrame0::SetBrake((false));
    }else{
        DriverControlsFrame0::SetBrake((false));
    }
    //CANController::Send(&DriverControlsFrame0::Instance());

}




