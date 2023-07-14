#include "user.hpp"
#include <stddef.h>


extern "C" void CPP_UserSetup(void);

void CPP_UserSetup(void) {
    while (1) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(1000);
    }
}