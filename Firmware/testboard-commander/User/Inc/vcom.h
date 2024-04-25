#pragma once

#include "cmsis_os2.h"

// Size of command buffer
#define CMD_MAX_SIZE 256

// Queue to hold commands
extern osMessageQueueId_t command_queue;

// Initialize virtual COM port
void VCOM_Init(void);

// Transmit data over virtual COM port
void VCOM_Transmit(const char* buf, uint32_t len);