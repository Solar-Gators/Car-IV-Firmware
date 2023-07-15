#include "user.hpp"


extern "C" void CPP_UserSetup(void);

void CPP_UserSetup(void) {
    while (1) {
        SolarGators::Logger::LogInfo("This is an info: %u\n", 5);
        SolarGators::Logger::LogDebug("This is a debug\n");
        SolarGators::Logger::LogWarning("This is a warning\n");
        SolarGators::Logger::LogError("This is an error\n");

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(100);
    }
}