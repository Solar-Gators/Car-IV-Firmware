#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "main.h"
#include "SEGGER_RTT.h"

class Logger {
public:
    static void LogInfo(const char * sFormat, ...);
    static void LogDebug(const char * sFormat, ...);
    static void LogWarning(const char * sFormat, ...);
    static void LogError(const char * sFormat, ...);
};

#endif  /* LOGGER_HPP_ */