/*
 *  Sample Implementation:
 *  LongButton button1 = LongButton(BTN1_GPIO_Port,BTN1_Pin);
 *  button1.press_callback_ = PressCallback;
 *  button1.release_callback_ = ReleaseCallback;
 * 
 */
#include "longpressbutton.hpp"
#include "logger.hpp"

extern "C" TIM_HandleTypeDef htim4;
void Event(void *argument);
LongButton::LongButton(GPIO_TypeDef *port, uint16_t pin, 
                uint32_t debounce_time_ms,
                GPIO_PinState default_state, 
                bool initial_toggle_state):
    port_(port), pin_(pin), debounce_time_ms_(debounce_time_ms), 
    default_state_(default_state), toggle_state_(initial_toggle_state) {

        //Create new thread: 1 thread per long button
        handle_press_task_id_ = osThreadNew((osThreadFunc_t)&Event, NULL, &handle_press_task_attributes_);

        // Make sure button pin is not already in use
        for (auto button : long_button_list_) {
            if (button->GetPin() == pin_) {
                Logger::LogError("Duplicate button pin");
                Error_Handler();
            }
        }
    
    // Add button to button list
    long_button_list_.push_back(this);
};

void LongButton::RegisterPressCallback(void (*callback)(void)) {
    press_callback_ = callback;
}

void LongButton::RegisterReleaseCallback(void (*callback)(void), uint32_t check_release_time) {
    release_callback_ = callback;
    check_release_time_ = check_release_time;
}

uint32_t LongButton::GetPin() {
    return pin_;
}

bool LongButton::GetToggleState() {
    return toggle_state_;
}

GPIO_PinState LongButton::ReadPin() {
    return HAL_GPIO_ReadPin(port_, pin_);
}

void Event(void *argument) {
    while (1) {
        LongButton button = *(LongButton*)argument;
        // Wait for button press
        osSemaphoreAcquire(button.longbutton_semaphore_id_, osWaitForever);

        // Disable pin interrupt
        button.DisableInterrupt();

        button.toggle_state_=!button.toggle_state_;

        //Run the press callback function:
        button.press_callback_();

        // Wait for debounce period
        osDelay(button.debounce_time_ms_);

        // If button is still pressed, button is currently in the pressed position
        if (button.ReadPin() != button.default_state_) {
            button.PollForRelease();
        }

        // Enable pin interrupt
        button.ClearInterrupt();
        button.EnableInterrupt();
    }
}

void LongButton::PollForRelease() {
    while(1){
        // Repeat after check_release_time
        osDelay(check_release_time_);

        // If button is released, normalpress has occurred
        if (ReadPin() == default_state_) {
            toggle_state_=default_state_;
            release_callback_();
            return;
        }
    }
}

void LongButton::DisableInterrupt() {
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

void LongButton::EnableInterrupt() {
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

void LongButton::ClearInterrupt() {
    __HAL_GPIO_EXTI_CLEAR_FLAG(pin_);
}

void LongButton::TriggerButton(uint16_t GPIO_Pin){
    for(auto button : LongButton::long_button_list_){
        if(button->GetPin()==GPIO_Pin){
            osSemaphoreRelease(button->longbutton_semaphore_id_);
            return;
        }
    }
}