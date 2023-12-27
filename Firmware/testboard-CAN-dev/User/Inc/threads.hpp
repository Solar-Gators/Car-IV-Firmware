#pragma once

#include "user.hpp"

/* Thread function prototypes */
void PeriodicThread(void *argument);

/* Periodic threads */
extern osTimerId_t periodic_thread_id;