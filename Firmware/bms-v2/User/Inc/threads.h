#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

void ThreadsStart();

/* Defines */


/* Task function prototypes */
void ReadVoltageThread(void *argument);
void ReadCurrentThread(void *argument);
void ReadTemperatureThread(void *argument);
void BroadcastThread(void *argument);

/* Periodic threads */
extern osTimerId_t voltage_timer_id;
extern osTimerId_t current_timer_id;
extern osTimerId_t temperature_timer_id;
extern osTimerId_t broadcast_timer_id;

/* Regular task definitions */


/* Event flags */

/* Mutexes */

/* Callbacks */
//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif  /* THREADS_H_ */