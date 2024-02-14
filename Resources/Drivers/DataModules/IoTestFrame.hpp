/*
 * IoTestFrame.hpp
 *
 *  Created on: February 12, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include <stdint.h>
#include "DataModule.hpp"

DATAMODULE(IoTestFrame, 0x37, CAN_ID_STD, CAN_RTR_DATA, 8,
    inline static GPIO_PinState GetOkLed() {
        return (GPIO_PinState)Data()[0];
    }

    inline static GPIO_PinState GetErrorLed() {
        return (GPIO_PinState)Data()[1];
    }

    inline static void SetOkLed(GPIO_PinState state) {
        Data()[0] = state;
    }

    inline static void SetErrorLed(GPIO_PinState state) {
        Data()[1] = state;
    }
)