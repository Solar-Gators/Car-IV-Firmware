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

#if (USE_FREERTOS == 1)
#include "cmsis_os.h"
#endif

#define LOG_LEVEL_INFO 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_WARNING 2
#define LOG_LEVEL_ERROR 3
#define LOG_LEVEL_NONE 4

class Logger {
public:
    static void LogDebug(const char * sFormat, ...);
    static void LogInfo(const char * sFormat, ...);
    static void LogWarning(const char * sFormat, ...);
    static void LogError(const char * sFormat, ...);
private:
    static void PrintHeader(const char * sLevel);
    static void PrintThread();
    static void LogCommon(const char * sFormat, ...);
};

#endif  /* LOGGER_HPP_ */