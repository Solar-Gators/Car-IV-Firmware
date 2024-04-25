#include "vcom.h"

#include <stdint.h>
#include "usbd_cdc_if.h"
#include "logger.hpp"

static char command_buf[CMD_MAX_SIZE];
static char* pbuf = command_buf;
osMessageQueueId_t command_queue;

void VCOM_Init(void) {
    Logger::LogInfo("VCOM initialized\n");
    command_queue = osMessageQueueNew(2, sizeof(char)*CMD_MAX_SIZE, NULL);
}

void VCOM_Transmit(const char* buf, uint32_t len) {
    while (CDC_Transmit_FS((uint8_t*)buf, len) != USBD_OK);
}

extern "C" void CDC_RxCallback(uint8_t *buf, uint32_t len) {
    // Echo received data
    while (CDC_Transmit_FS(buf, len) != USBD_OK);

    // Put received data into buffer
    for (uint32_t i = 0; i < len; i++) {
        switch (buf[i]) {
            case '\r':
                break;
            case '\n':
                *pbuf = '\0';
                
                // Put buffer into queue
                osMessageQueuePut(command_queue, command_buf, 0, 0);

                // Reset buffer to start
                pbuf = command_buf;
                break;
            default:
                *pbuf = buf[i];
                pbuf++;

                // If buffer is full, reset pointer to start
                if (pbuf >= command_buf + CMD_MAX_SIZE)
                    pbuf = command_buf;
                break;
        }
    }

    Logger::LogDebug("Received string: %s", command_buf);
}