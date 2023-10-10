#include "balancer.hpp"

/**
  * @brief  Constructor for Balancer class.
  * @param  send_htim Pointer to timer handle for send channel.
  * @param  send_channel Channel for send channel.
  * @param  recv_htim Pointer to timer handle for receive channel.
  * @param  recv_channel Channel for receive channel.
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::Init(TIM_HandleTypeDef* send_htim, uint32_t send_channel,
                                TIM_HandleTypeDef* recv_htim , uint32_t recv_channel) {
    this->send_htim = send_htim;
    this->recv_htim = recv_htim;
    this->send_channel = send_channel;
    this->recv_channel = recv_channel;

    // Send and receive are complements on the same channel
    if (send_htim == recv_htim && (send_channel - recv_channel == 1 || recv_channel - send_channel == 1)) {
        this->adjacent_channels = true;
    } else {
        this->adjacent_channels = false;
    }

    return HAL_OK;
}

/**
  * @brief  Set frequency of send and receive timers.
  * @param  frequency Frequency in Hz.
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::SetFrequency(uint32_t frequency) {
    TIM_Base_InitTypeDef sConfig;
    sConfig.Prescaler = 0;
    sConfig.CounterMode = TIM_COUNTERMODE_UP;
    sConfig.Period = SystemCoreClock / frequency;
    sConfig.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.RepetitionCounter = 0;
    sConfig.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    this->send_htim->Init = sConfig;
    this->recv_htim->Init = sConfig;

    return HAL_TIM_Base_Init(this->send_htim);
}

/**
  * @brief  Sets the period of the send / recv channel pair.
  * @param  period Period in ns.
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::SetPeriod(uint32_t period) {
    TIM_Base_InitTypeDef sConfig;
    sConfig.Prescaler = 0;
    sConfig.CounterMode = TIM_COUNTERMODE_UP;
    sConfig.Period = (uint32_t)(((uint64_t)SystemCoreClock * period) / 1000000000);
    sConfig.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.RepetitionCounter = 0;
    sConfig.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    this->send_htim->Init = sConfig;
    this->recv_htim->Init = sConfig;

    return HAL_TIM_Base_Init(this->send_htim);
}

/**
  * @brief  Sets the duty cycle of the send / recv channel pair.
  * @param  ratio Duty cycle of send channel out of 100.
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::SetRatio(uint32_t ratio) {
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // Send and receive on same channel, send on CHX, receive on CHXN
    if (send_htim == recv_htim && recv_channel - send_channel == 1) {
        sConfigOC.Pulse = 0;
        sConfigOC.Pulse = ratio * this->send_htim->Init.Period / 100;
        HAL_StatusTypeDef status = HAL_TIM_PWM_ConfigChannel(this->send_htim, &sConfigOC, this->send_channel);
        if (status != HAL_OK) { return status; }
        return HAL_OK;
    }
    // Send and receive on same channel, send on CHXN, receive on CHX
    if (send_htim == recv_htim && send_channel - recv_channel == 1) {
        sConfigOC.Pulse = (100 - ratio) * this->send_htim->Init.Period / 100;
        HAL_StatusTypeDef status = HAL_TIM_PWM_ConfigChannel(this->send_htim, &sConfigOC, this->recv_channel);
        if (status != HAL_OK) { return status; }
        return HAL_OK;
    }

    // TODO: Add support for different channel configurations
    return HAL_ERROR;
}

/**
  * @brief  Sets the dead time of the send / recv channel pair.
  * @param  deadtime Dead time in ns.
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::SetDeadTime(uint32_t deadtime) {

    uint64_t deadtime_ticks = (uint64_t)SystemCoreClock;
    deadtime_ticks *= deadtime;
    deadtime_ticks /= 1000000000;

    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
    sBreakDeadTimeConfig.DeadTime = (uint32_t)deadtime_ticks;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;

    // TODO: Add support for different channel configurations
    return HAL_TIMEx_ConfigBreakDeadTime(this->send_htim, &sBreakDeadTimeConfig);
}

/**
  * @brief  Starts balancer
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::Start() {
    HAL_StatusTypeDef status = HAL_OK;

    if (this->adjacent_channels) {
        uint32_t primary_channel = (send_channel < recv_channel) ? send_channel : recv_channel;
        status = HAL_TIM_PWM_Start(this->send_htim, primary_channel);
        if (status != HAL_OK) { return status; }

        // Wait for timer to trigger before starting complementary channel
        this->send_htim->Instance->SR &= ~TIM_SR_UIF;
        while (!(this->send_htim->Instance->SR & TIM_SR_UIF));

        status = HAL_TIMEx_PWMN_Start(this->send_htim, primary_channel);
        if (status != HAL_OK) { return status; }
        return HAL_OK;
    }

    // TODO: Add support for different channel configurations
    return HAL_ERROR;
}

/**
  * @brief  Stops balancer
  * @retval HAL Status
  */
HAL_StatusTypeDef Balancer::Stop() {
    HAL_StatusTypeDef status = HAL_OK;

    if (this->adjacent_channels) {
        uint32_t primary_channel = (send_channel < recv_channel) ? send_channel : recv_channel;

        TIM_TypeDef* instance = this->send_htim->Instance;

        // Disable complementary channel first
        instance->CCER &= (~primary_channel & 0x1FU);

        // Wait for primary channel to go low
        while (instance->CNT < instance->CCR1);

        // Disable timer, then disable capture/compare
        this->send_htim->Instance->CR1 &= ~TIM_CR1_CEN;
        this->send_htim->Instance->CCER &= (~(primary_channel << 2) & 0x1FU);

        // status = HAL_TIM_PWM_Stop(this->send_htim, primary_channel);
        // if (status != HAL_OK) { return status; }
        // status = HAL_TIMEx_PWMN_Stop(this->send_htim, primary_channel);
        // if (status != HAL_OK) { return status; }

        return HAL_OK;
    }

    // TODO: Add support for different channel configurations
    return HAL_ERROR;
}