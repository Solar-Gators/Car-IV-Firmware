#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

/* Defines */


/* Task function prototypes */
void JoystickTask(void *argument);
void SDTask(void *argument);
void VoiceTask(void *argument);
void MenuTask(void *argument);
void InputHandlerTask(void *argument);

void MotorUpdateTask(void *argument);

/* Periodic threads */
extern osTimerId_t joystick_timer_id;

/* Regular task definitions */
extern osThreadId_t sd_task_id;
extern osThreadId_t voice_task_id;
extern osThreadId_t menu_task_id;
extern osThreadId_t input_task_id;

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