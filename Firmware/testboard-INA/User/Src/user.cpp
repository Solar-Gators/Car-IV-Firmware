#include "user.hpp"

extern "C" void CPP_UserSetup(void);

void CPP_UserSetup(void) {
    INA226 INA;

    HAL_StatusTypeDef status;
    status = INA.Init(&hi2c2);
    if(status != HAL_OK) while(1);

    INA.SetConfig();

    INA.SetCalibrationReg(0.02, 5);

    while(1){
        status = INA.GetShuntVoltage();
        if(status != HAL_OK) while(1);
        HAL_Delay(500);

				status = INA.GetBusVoltage();
				if(status != HAL_OK) while(1);
        HAL_Delay(500);

        status = INA.GetCurrent();
        if(status != HAL_OK) while(1);
        HAL_Delay(500);
    }
}