#include "main.h"


/*
 * TODO: Add different channel configurations
 * TODO: Reference clock to AHB clock instead of system cloc
*/

/* Complementary channels are HAL channel definitions incremented by 1 */
#define TIM_CHANNEL_1N                     0x00000001U                          /*!< Capture/compare channel 1N identifier      */
#define TIM_CHANNEL_2N                     0x00000005U                          /*!< Capture/compare channel 2N identifier      */
#define TIM_CHANNEL_3N                     0x00000009U                          /*!< Capture/compare channel 3N identifier      */
#define TIM_CHANNEL_4N                     0x0000000DU                          /*!< Capture/compare channel 4N identifier      */
#define TIM_CHANNEL_5N                     0x00000011U                          /*!< Compare channel 5N identifier              */
#define TIM_CHANNEL_6N                     0x00000015U                          /*!< Compare channel 6N identifier              */

class Balancer {
public:
    HAL_StatusTypeDef Init(TIM_HandleTypeDef* send_htim, uint32_t send_channel,
                            TIM_HandleTypeDef* recv_htim , uint32_t recv_channel);
    HAL_StatusTypeDef SetFrequency(uint32_t frequency);
    HAL_StatusTypeDef SetPeriod(uint32_t period);
    HAL_StatusTypeDef SetRatio(uint32_t ratio);
    HAL_StatusTypeDef SetDeadTime(uint32_t deadtime);
    HAL_StatusTypeDef Start();
    HAL_StatusTypeDef Stop();
private:
    TIM_HandleTypeDef* send_htim;
    TIM_HandleTypeDef* recv_htim;
    uint32_t send_channel;
    uint32_t recv_channel;

    bool adjacent_channels;
};