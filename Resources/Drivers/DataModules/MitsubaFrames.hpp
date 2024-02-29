/*
 * MitsubaFrames.hpp
 *
 *  Created on: February 28, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

DATAMODULE(

    /* CAN definitions */
    MitsubaRequestFrame,
    0x08F89540,
    CAN_ID_EXT,
    CAN_RTR_DATA,
    8,

    inline static void SetRequestAll() {
        Data()[0] |= 0x3;
    }

    inline static void SetRequest0() {
        Data()[0] |= (0x1 << 0);
    }

    inline static void SetRequest1() {
        Data()[0] |= (0x1 << 1);
    }

    inline static void SetRequest2() {
        Data()[0] |= (0x1 << 2);
    }

    inline static void ResetRequest0() {
        Data()[0] &= ~(0x1 << 0);
    }

    inline static void ResetRequest1() {
        Data()[0] &= ~(0x1 << 1);
    }

    inline static void ResetRequest2() {
        Data()[0] &= ~(0x1 << 2);
    }
)

DATAMODULE(

    MitsubaFrame0,
    0x08850225,
    CAN_ID_EXT,
    CAN_RTR_DATA,
    8,

)