#pragma once

#include "user.hpp"

void ThreadsInit(void);

/* Thread function prototypes */
void PeriodicThread(void *argument);
void ProcessCommand(void* argument);

/* Periodic threads */
extern osTimerId_t periodic_thread_id;
extern osThreadId_t command_thread_id;

/* Callbacks */
void ButtonSingleCallback(void);
void ButtonLongCallback(void);
void IoMsgCallback(uint8_t *data);
void MitsubaMsgCallback(uint8_t *data);