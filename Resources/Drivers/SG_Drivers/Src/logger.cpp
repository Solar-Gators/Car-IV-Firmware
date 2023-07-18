#include "logger.hpp"

void Logger::LogInfo(const char * sFormat, ...) {
    uint32_t timestamp = HAL_GetTick();
    SEGGER_RTT_printf(0, "%10u [INFO] ", timestamp);

    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
}

void Logger::LogDebug(const char * sFormat, ...) {
    uint32_t timestamp = HAL_GetTick();
    SEGGER_RTT_printf(0, "%10u [DEBUG] ", timestamp);

    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
}

void Logger::LogWarning(const char * sFormat, ...) {
    uint32_t timestamp = HAL_GetTick();
    SEGGER_RTT_printf(0, "%10u [WARNING] ", timestamp);

    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
}

void Logger::LogError(const char * sFormat, ...) {
    uint32_t timestamp = HAL_GetTick();
    SEGGER_RTT_printf(0, "%10u [ERROR] ", timestamp);

    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
}