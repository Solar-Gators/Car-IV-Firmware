#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

/* Task function prototypes */
void PeriodicTask1(void *argument);
void VoiceTask(void *argument);
void MenuTask(void *argument);
void UITask(void *argument);

/* Periodic threads */
extern osTimerId_t periodic_timer_id;

/* Regular task definitions */
extern osThreadId_t voice_task_id;
extern osThreadId_t menu_task_id;
extern osThreadId_t ui_task_id;

/* Event flags */
extern osEventFlagsId_t regular_event;

/* Mutexes */
extern osMutexId_t spi_mutex_id;

/* ISRs */
//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif  /* THREADS_H_ */