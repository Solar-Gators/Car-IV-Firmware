#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

/* Defines */


/* Task function prototypes */

/* Periodic threads */
void SendMitsubaRequest(void);
void ToggleLights(void);
void LogData(void);

/* Regular task definitions */
void ThreadsStart();

/* Event flags */

/* Mutexes */

/* Callbacks */
void IoMsgCallback(uint8_t *data);
void DriverControls0Callback(uint8_t *data);
void DriverControls1Callback(uint8_t *data);
void MitsubaCallback(uint8_t *data);
void KillSwitchCallback(void);


#endif  /* THREADS_H_ */