#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

void ThreadsStart();

/* Defines */


/* Task function prototypes */
void ReadVoltageThread(void *argument);
void ReadCurrentThread(void *argument);
void ReadTemperatureThread(void *argument);

/* Periodic threads */
extern osTimerId_t voltage_timer_id;

/* Regular task definitions */


/* Event flags */
extern osEventFlagsId_t regular_event;
extern osEventFlagsId_t filename_event; // SD file open request
extern osEventFlagsId_t lineread_event; // SD line read request/response

/* Mutexes */
extern osMutexId_t spi_mutex_id;
extern osMutexId_t text_mutex_id;

/* Callbacks */
//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif  /* THREADS_H_ */