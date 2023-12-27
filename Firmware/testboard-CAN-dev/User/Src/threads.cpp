#include "threads.hpp"

/* Periodic thread definitions */
osTimerAttr_t periodic_thread_attr = {
    .name = "Periodic Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_thread_id = osTimerNew((osTimerFunc_t)PeriodicThread, osTimerPeriodic, NULL, &periodic_thread_attr);


/* Thread function definitions */
void PeriodicThread(void *argument) {
    Logger::LogInfo("Periodic thread running\n");

    #if (SENDER_NODE == 1)
    //CANController::Send(&msg1_static);
    #endif
}