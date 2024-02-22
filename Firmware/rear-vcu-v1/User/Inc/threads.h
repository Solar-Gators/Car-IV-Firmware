#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

/* Defines */


/* Task function prototypes */


/* Periodic threads */

/* Regular task definitions */

/* Event flags */


/* Mutexes */

/* Callbacks */
//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void IoMsgCallback(uint8_t *data);
void MotorUpdateCallback(uint8_t *data);


#endif  /* THREADS_H_ */