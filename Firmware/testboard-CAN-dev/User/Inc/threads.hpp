#pragma once

#include "user.hpp"

/* Thread function prototypes */
void PeriodicThread(void *argument);

/* Periodic threads */
extern osTimerId_t periodic_thread_id;

/* Callbacks */
void ButtonSingleCallback(void);
void ButtonLongCallback(void);
void IoMsgCallback(uint8_t *data);