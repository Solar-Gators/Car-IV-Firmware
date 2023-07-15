#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "SEGGER_RTT.h"
#include "stm32l4xx_hal.h"

#define ANSI_RED "\033[4;41m"
#define ANSI_RESET "\033[0m"

namespace SolarGators {

class Logger {
public:
    static void LogInfo(const char * sFormat, ...);
    static void LogDebug(const char * sFormat, ...);
    static void LogWarning(const char * sFormat, ...);
    static void LogError(const char * sFormat, ...);
};

}   /* namesapce SolarGators */

#endif  /* LOGGER_HPP_ */