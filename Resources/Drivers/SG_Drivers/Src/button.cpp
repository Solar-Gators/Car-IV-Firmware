#include "button.hpp"

extern "C" TIM_HandleTypeDef htim4;

Button::Button(GPIO_TypeDef *port, uint16_t pin, 
                uint32_t debounce_time_ms,
                GPIO_PinState default_state, 
                bool initial_toggle_state):
    port_(port), pin_(pin), debounce_time_ms_(debounce_time_ms), 
    default_state_(default_state), toggle_state_(initial_toggle_state) {

    // Make sure button pin is not already in use
    for (auto button : button_list_) {
        if (button->GetPin() == pin_) {
            Logger::LogError("Duplicate button pin");
            Error_Handler();
        }
    }
    
    // Add button to button list
    button_list_.push_back(this);

    // Clear
    last_press_time_ = 0;
    long_press_time_ms_ = 0;        // Long press disabled
    double_press_time_ms_ = 0;      // Double press disabled
};

void Button::RegisterNormalPressCallback(void (*callback)(void)) {
    normal_callback_ = callback;
}

void Button::RegisterLongPressCallback(void (*callback)(void), uint32_t long_press_time_ms) {
    long_press_callback_ = callback;
    long_press_time_ms_ = long_press_time_ms;
}

void Button::RegisterDoublePressCallback(void (*callback)(void), uint32_t double_press_time_ms) {
    double_press_callback_ = callback;
    double_press_time_ms_ = double_press_time_ms;
}

uint32_t Button::GetPin() {
    return pin_;
}

GPIO_PinState Button::ReadPin() {
    return HAL_GPIO_ReadPin(port_, pin_);
}

void Button::HandleEvent() {
    while (1) {
        // Wait for button press
        osSemaphoreAcquire(button_semaphore_id_, osWaitForever);

        // Store button locally since another interrupt may occur during debounce
        // and triggered_button_ will be overwritten
        Button *button = triggered_button_;

        // Disable pin interrupt
        button->DisableInterrupt();

        // Wait for debounce period
        osDelay(button->debounce_time_ms_);

        // If button is still pressed, button is currently in the pressed position
        if (button->ReadPin() != button->default_state_) {
            // If long press is enabled, start polling button
            if (button->long_press_callback_ != NULL)
                button->PollForLongPress();
            // If long press is not enabled, call HandlePress()
            else
                button->HandlePress();
        }

        // Enable pin interrupt
        button->ClearInterrupt();
        button->EnableInterrupt();
    }
}

void Button::PollForLongPress() {
    for (uint32_t i = 25; i < long_press_time_ms_; i += 25) {
        // Wait for 25 ms
        osDelay(25);

        // If button is released, normalpress has occurred
        if (ReadPin() == default_state_) {
            HandlePress();
            return;
        }
    }

    // If we get here, long press has occurred
    Logger::LogInfo("Long Press");
    long_press_callback_();
}

void Button::HandlePress() {
    // If double press is enabled
    if (double_press_callback_ != NULL) {
        // Double press has occurred
        if (osKernelGetTickCount() - last_press_time_ < double_press_time_ms_) {
            Logger::LogInfo("Double Press");
            double_press_callback_();
            last_press_time_ = 0;
            return;
        }
    }
    // If we get here double press has not occurred
    Logger::LogInfo("Normal Press");
    last_press_time_ = osKernelGetTickCount();
    normal_callback_();
}

void Button::DisableInterrupt() {
    switch (pin_) {
        case GPIO_PIN_0:
            HAL_NVIC_DisableIRQ(EXTI0_IRQn);
            break;
        case GPIO_PIN_1:
            HAL_NVIC_DisableIRQ(EXTI1_IRQn);
            break;
        case GPIO_PIN_2:
            HAL_NVIC_DisableIRQ(EXTI2_IRQn);
            break;
        case GPIO_PIN_3:
            HAL_NVIC_DisableIRQ(EXTI3_IRQn);
            break;
        case GPIO_PIN_4:
            HAL_NVIC_DisableIRQ(EXTI4_IRQn);
            break;
        case GPIO_PIN_5:
        case GPIO_PIN_6:
        case GPIO_PIN_7:
        case GPIO_PIN_8:
        case GPIO_PIN_9:
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            break;
        case GPIO_PIN_10:
        case GPIO_PIN_11:
        case GPIO_PIN_12:
        case GPIO_PIN_13:
        case GPIO_PIN_14:
        case GPIO_PIN_15:
            HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
            break;
        default:
            break;
    }
}

void Button::EnableInterrupt() {
    switch (pin_) {
        case GPIO_PIN_0:
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
            break;
        case GPIO_PIN_1:
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
            break;
        case GPIO_PIN_2:
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);
            break;
        case GPIO_PIN_3:
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            break;
        case GPIO_PIN_4:
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
            break;
        case GPIO_PIN_5:
        case GPIO_PIN_6:
        case GPIO_PIN_7:
        case GPIO_PIN_8:
        case GPIO_PIN_9:
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            break;
        case GPIO_PIN_10:
        case GPIO_PIN_11:
        case GPIO_PIN_12:
        case GPIO_PIN_13:
        case GPIO_PIN_14:
        case GPIO_PIN_15:
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
            break;
        default:
            break;
    }
}

void Button::ClearInterrupt() {
    __HAL_GPIO_EXTI_CLEAR_FLAG(pin_);
}

// Global interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    for (auto button : Button::button_list_) {
        if (button->GetPin() == GPIO_Pin) {
            Button::triggered_button_ = button;
            osSemaphoreRelease(Button::button_semaphore_id_);
        }
    }
}