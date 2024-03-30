#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

void ThreadsStart();

/* Defines */

/* Periodic threads */
void SendMitsubaRequest(void);
void ToggleLights(void);
void LogDataPeriodic(void);

/* Regular threads */
void LogData(void);

/* Event flags */

/* Mutexes */

/* Callbacks */
void IoMsgCallback(uint8_t *data);
void DriverControls0Callback(uint8_t *data);
void DriverControls1Callback(uint8_t *data);
void BMSFaultCallback(uint8_t *data);
void MitsubaCallback(uint8_t *data);
void KillSwitchCallback(void);


#endif  /* THREADS_H_ */