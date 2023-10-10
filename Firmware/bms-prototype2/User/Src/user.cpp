#include "user.hpp"


extern "C" void CPP_UserSetup(void);

/* Active Balancer Pinout
 * Send8 - PA9 - TIM1_CH2
 * Recv8 - PC7 - TIM3_CH2
 * Send7 - PB1 - TIM1_CH3N
 * Recv7 - PA8 - TIM1_CH1
 * Send2 - PB10 - TIM2_CH3
 * Recv2 - PB14 - TIM1_CH2N
 * Send1 - PB13 - TIM1_CH1N
 * Recv1 - PA10 - TIM1_CH3
 *
 * Send1 -> Recv7 (TIM1_CH1N -> TIM1_CH1)
 * Send2 -> Recv8 (TIM2_CH3 -> TIM2_CH2)
 * Send7 -> Recv1 (TIM1_CH3N -> TIM1_CH3)
 * Send8 -> Recv2 (TIM1_CH2 -> TIM1_CH2N) x
*/

void CPP_UserSetup(void) {
    // Balancer bal8to2;
    // bal8to2.Init(&htim1, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_2N);
    // bal8to2.SetPeriod(3000);
    // bal8to2.SetRatio(83);
    // bal8to2.SetDeadTime(150);

    // bal8to2.Start();
    // HAL_Delay(1);
    // bal8to2.Stop();
    

    while (1) {
        // directCommand(0x14)
        uint8_t cell1_data = 0x14;
        HAL_I2C_Master_Transmit(&hi2c1, 0x10, &cell1_data, 1, HAL_MAX_DELAY);
        
        uint8_t data[2];
        HAL_I2C_Master_Receive(&hi2c1, 0x11, data, 2, HAL_MAX_DELAY);

        Logger::LogInfo("Cell 1: %d", data[0]);
        Logger::LogInfo("Cell 1: %d", data[1]);
    }
}