/*
 * DataModule.hpp
 *
 *  Created on: February 12, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "sg_can.hpp"

// This macro acts as a singleton wrapper for CANFrames to prevent errors from multiple CANFrame instances
// Refer to other files in this directory for usage examples

#define DATAMODULE(CLASS_NAME, CAN_ID, CAN_ID_TYPE, CAN_RTR_MODE, CAN_DATA_BYTES, FUNCTIONS) \
class CLASS_NAME : public CANFrame { \
    private: \
        CLASS_NAME() : CANFrame(CAN_ID, CAN_ID_TYPE, CAN_RTR_MODE, CAN_DATA_BYTES) {} \
        ~CLASS_NAME() {} \
    public: \
        static CLASS_NAME& Instance() { \
            static CLASS_NAME instance; \
            return instance; \
        } \
        CLASS_NAME(const CLASS_NAME&) = delete; \
        CLASS_NAME& operator=(const CLASS_NAME&) = delete; \
        static uint8_t* Data() { \
            return Instance().data; \
        } \
    FUNCTIONS \
};

// Prevent the compiler from complaining about backslash-newline at end of file
inline int stuff;