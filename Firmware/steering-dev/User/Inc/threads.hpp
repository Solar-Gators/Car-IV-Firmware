#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

/* Defines */

/* Periodic threads */
void ReadButtonsPeriodic();

/* Regular threads */
void TurnSignalToggle(void);

/* Event flags */
extern osEventFlagsId_t turn_signal_event;

/* Mutexes */

/* Callbacks */
void RightTurnCallback(void);
void CruisePlusCallback(void);
void CruiseMinusCallback(void);
void PTTCallback(void);

void ThreadsStart(void);


#endif  /* THREADS_H_ */