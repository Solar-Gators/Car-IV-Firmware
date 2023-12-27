/*
 * button.hpp
 *
 *  Created on: December 25, 2023
 *      Author: Matthew Shen
 * 
 * 
 *  Usage Example:
 *  Button user_button = Button(BTN1_GPIO_Port, BTN1_Pin, 50, GPIO_PIN_SET, false);
 *  button.RegisterNormalPressCallback(ButtonCallback);
 *  button.RegisterLongPressCallback(LongCallback, 300);
 *  button.RegisterDoublePressCallback(DoubleCallback, 300);
 */

#pragma once

#include "user.hpp"
#include "logger.hpp"
#include "etl/vector.h"

/* Default constant values, define in user_config if different values desired*/
#ifndef MAX_BUTTONS
#define MAX_BUTTONS 10
#endif

#ifndef BUTTON_THREAD_STACK_SIZE
#define BUTTON_THREAD_STACK_SIZE 256
#endif

class Button {
public:
    Button(GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_time_ms = 50,
             GPIO_PinState default_state = GPIO_PIN_SET, bool initial_toggle_state = false);
    void RegisterNormalPressCallback(void (*callback)(void));
    void RegisterLongPressCallback(void (*callback)(void), 
                                        uint32_t long_press_time_ms = 500, 
                                        bool initial_toggle_state = false);
    void RegisterDoublePressCallback(void (*callback)(void),
                                        uint32_t double_press_time_ms = 500,
                                        bool initial_toggle_state = false);

    uint32_t GetPin();
    GPIO_PinState ReadPin();
    bool GetToggleState();
    bool GetLongToggleState();
    bool GetDoubleToggleState();

    // Current triggered button
    static inline Button *triggered_button_;

    // Button trigger semaphore
    static inline osSemaphoreId_t button_semaphore_id_ = osSemaphoreNew(1, 0, NULL);
     
    // Global button list
    static inline etl::vector<Button*, MAX_BUTTONS> button_list_;

private:
    // Button handler thread definitions
    static void HandleEvent();
    static inline uint32_t handle_press_task_buffer_[BUTTON_THREAD_STACK_SIZE];
    static inline StaticTask_t handle_press_task_tcb_;
    static constexpr const osThreadAttr_t handle_press_task_attributes_ = {
        .name = "handle_press_task",
        .attr_bits = osThreadDetached,
        .cb_mem = &handle_press_task_tcb_,
        .cb_size = sizeof(handle_press_task_tcb_),
        .stack_mem = &handle_press_task_buffer_[0],
        .stack_size = sizeof(handle_press_task_buffer_),
        .priority = (osPriority_t) osPriorityAboveNormal,
        .tz_module = 0,
        .reserved = 0,
    };
    static inline osThreadId_t handle_press_task_id_ = osThreadNew((osThreadFunc_t)HandleEvent, NULL, &handle_press_task_attributes_);

    // Private functions
    void DisableInterrupt();
    void EnableInterrupt();
    void ClearInterrupt();

    void PollForLongPress();
    void HandlePress();

    // Peripheral information
    GPIO_TypeDef *port_;            // HAL GPIO port
    uint16_t pin_;                  // Pin number
    uint32_t debounce_time_ms_;     // Debounce time in ms
    GPIO_PinState default_state_;   // Default pin state
    bool toggle_state_;             // Toggle state of button, toggled on single and double press
    bool long_toggle_state_;        // Toggle state of button, toggled on long press
    bool double_toggle_state_;      // Toggle state of button, toggled on double press

    void (*normal_callback_)(void);        // User-provided callback function
    void (*long_press_callback_)(void);    // User-provided callback function for long press
    void (*double_press_callback_)(void);  // User-provided callback function for double press

    uint32_t long_press_time_ms_;   // Time to count as a long press
    uint32_t double_press_time_ms_; // Time between presses to count as a double press

    // Double press state information
    uint32_t last_press_time_;     // Last time valid event was recorded on button
};
