#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

/* Defines */

/* Periodic threads */
void ReadButtonsPeriodic();
void UpdateUIPeriodic();

/* Regular threads */
void TurnSignalToggle(void);

/* Event flags */
extern osEventFlagsId_t turn_signal_event;

/* Mutexes */
extern osMutexId_t ui_mutex;

/* Callbacks */
void LeftTurnCallback(void);
void ModeCallback(void);
void ModeLongCallback(void);
void RegenCallback(void);
void HornCallback(void);
void MCCallback(void);
void RightTurnCallback(void);
void CruisePlusCallback(void);
void CruiseMinusCallback(void);
void PTTCallback(void);
void PVCallback(void);
void BMSResetCallback(void);

void BMSFrame3Callback(uint8_t *data);
void MitsubaCallback(uint8_t *data);

void ThreadsStart(void);


#endif  /* THREADS_H_ */