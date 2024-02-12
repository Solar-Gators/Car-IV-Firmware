/*
 * IoTestFrame.hpp
 *
 *  Created on: February 12, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include <stdint.h>
#include <sg_can.hpp>

class IoTestFrame : public CANFrame {
public:
    IoTestFrame() : CANFrame(0x37, CAN_ID_STD, CAN_RTR_DATA, 8) {}

    inline GPIO_PinState GetOkLed() {
        return (GPIO_PinState)this->data[0];
    }

    inline GPIO_PinState GetErrorLed() {
        return (GPIO_PinState)this->data[1];
    }

    inline void SetOkLed(GPIO_PinState state) {
        this->data[0] = state;
    }

    inline void SetErrorLed(GPIO_PinState state) {
        this->data[1] = state;
    }
};

inline IoTestFrame io_test_frame = IoTestFrame();