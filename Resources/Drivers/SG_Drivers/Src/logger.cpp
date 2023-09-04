#include "logger.hpp"

void Logger::LogInfo(const char * sFormat, ...) {
    PrintHeader("INFO");
    va_list ParamList;
    LogCommon(sFormat, ParamList);
}

void Logger::LogDebug(const char * sFormat, ...) {
    PrintHeader("DEBUG");
    va_list ParamList;
    LogCommon(sFormat, ParamList);
}

void Logger::LogWarning(const char * sFormat, ...) {
    PrintHeader("WARNING");
    va_list ParamList;
    LogCommon(sFormat, ParamList);
}

void Logger::LogError(const char * sFormat, ...) {
    PrintHeader("ERROR");
    va_list ParamList;
    LogCommon(sFormat, ParamList);
}

void Logger::PrintHeader(const char * sLevel) {
    uint32_t timestamp = HAL_GetTick();
    SEGGER_RTT_printf(0, "%10u [%s] ", timestamp, sLevel);
}

void Logger::PrintThread() {
#ifdef USE_FREERTOS
    SEGGER_RTT_printf(0, "[%s] ", osThreadGetName(osThreadGetId()));
#endif
}

void Logger::LogCommon(const char * sFormat, ...) {
    PrintThread();
    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
}