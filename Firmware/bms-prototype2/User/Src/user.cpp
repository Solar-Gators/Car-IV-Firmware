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
    Balancer bal8to2;
    bal8to2.Init(&htim1, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_2N);
    bal8to2.SetFrequency(1000000);
    bal8to2.SetRatio(17);
    bal8to2.SetDeadTime(30);
    bal8to2.Start();

    while (1) {
        
    }
}