#include "user.hpp"


extern "C" void CPP_UserSetup(void);

void CPP_UserSetup(void) {
    while (1) {
        HAL_I2C_Master_Transmit(&hi2c1, 0x00, 0x00, 0x00, 0x00);
    }
}