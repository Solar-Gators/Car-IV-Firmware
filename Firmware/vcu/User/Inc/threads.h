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
void StrobeThread(void);
void SendTelemetry(void);

/* Event flags */
extern osEventFlagsId_t log_event;
extern osEventFlagsId_t strobe_event;

/* Mutexes */

/* Callbacks */
void IoMsgCallback(uint8_t *data);
void DriverControls0Callback(uint8_t *data);
void DriverControls1Callback(uint8_t *data);
void BMSFaultCallback(uint8_t *data);
void MitsubaCallback(uint8_t *data);
void KillSwitchCallback(void);
void ThrottleTimeoutCallback(void);

/* Object Declarations */
transceiver TransceiverController(&huart1);

#endif  /* THREADS_H_ */