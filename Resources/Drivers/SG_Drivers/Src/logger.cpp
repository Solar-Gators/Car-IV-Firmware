#include "logger.hpp"

#if (USE_LOGGING == 1)
void Logger::LogInfo(const char * sFormat, ...) {
    #if (LOG_LEVEL <= LOG_LEVEL_INFO)
    PrintHeader("INFO");
    PrintThread();
    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
    SEGGER_RTT_printf(0, "\n");
    #endif
}

void Logger::LogDebug(const char * sFormat, ...) {
    #if (LOG_LEVEL <= LOG_LEVEL_DEBUG)
    PrintHeader("DEBUG");
    PrintThread();
    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
    SEGGER_RTT_printf(0, "\n");
    #endif
}

void Logger::LogWarning(const char * sFormat, ...) {
    #if (LOG_LEVEL <= LOG_LEVEL_WARNING)
    PrintHeader("WARNING");
    PrintThread();
    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
    SEGGER_RTT_printf(0, "\n");
    #endif
}

void Logger::LogError(const char * sFormat, ...) {
    #if (LOG_LEVEL <= LOG_LEVEL_ERROR)
    PrintHeader("ERROR");
    PrintThread();
    va_list ParamList;
    va_start(ParamList, sFormat);
    SEGGER_RTT_vprintf(0, sFormat, &ParamList);
    va_end(ParamList);
    SEGGER_RTT_printf(0, "\n");
    #endif
}
#else
void Logger::LogInfo(const char * sFormat, ...) {}
void Logger::LogDebug(const char * sFormat, ...) {}
void Logger::LogWarning(const char * sFormat, ...) {}
void Logger::LogError(const char * sFormat, ...) {}
#endif

void Logger::PrintHeader(const char * sLevel) {
    uint32_t timestamp = HAL_GetTick();
    SEGGER_RTT_printf(0, "%10u [%s] ", timestamp, sLevel);
}

void Logger::PrintThread() {
#if (USE_FREERTOS == 1)
    SEGGER_RTT_printf(0, "[%s] ", osThreadGetName(osThreadGetId()));
#endif
}