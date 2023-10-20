/*
 * logger.hpp
 *
 *  Created on: July 1, 2023
 *      Author: Matthew Shen
 */

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "main.h"
#include "user_config.h"
#include "SEGGER_RTT.h"

#ifdef USE_FREERTOS
#include "cmsis_os.h"
#endif

class Logger {
public:
    static void LogInfo(const char * sFormat, ...);
    static void LogDebug(const char * sFormat, ...);
    static void LogWarning(const char * sFormat, ...);
    static void LogError(const char * sFormat, ...);
private:
    static void PrintHeader(const char * sLevel);
    static void PrintThread();
    static void LogCommon(const char * sFormat, ...);
};

#endif  /* LOGGER_HPP_ */