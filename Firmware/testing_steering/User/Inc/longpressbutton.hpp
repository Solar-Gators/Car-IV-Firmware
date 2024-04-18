/*
 * longpressbutton.hpp
 *
 *  Created on: February 5, 2024
 *      Authors: Anthony Kfoury, Matthew Shen
 * 
 * 
 */

#pragma once

#include "user.hpp"
#include "logger.hpp"
#include "etl/vector.h"

#ifndef LONGBUTTON_THREAD_STACK_SIZE
#define LONGBUTTON_THREAD_STACK_SIZE 2048
#endif

/* Default constant values, define in user_config if different values desired*/
#ifndef MAX_LONGBUTTONS
#define MAX_LONGBUTTONS 5
#endif

class LongButton {
public:
    static void TriggerButton(uint16_t GPIO_Pin);
    LongButton(GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_time_ms = 50,
             GPIO_PinState default_state = GPIO_PIN_SET, bool initial_toggle_state = false);
    void RegisterPressCallback(void (*callback)(void));
    void RegisterReleaseCallback(void (*callback)(void),
                                        uint32_t check_release_time = 200);

    uint32_t GetPin();
    GPIO_PinState ReadPin();
    bool GetToggleState();

    void (*press_callback_)(void);        // User-provided callback function for button press
    void (*release_callback_)(void);      // User-provided callback function for button release

    static inline etl::vector<LongButton*, MAX_LONGBUTTONS> long_button_list_;

    // Button trigger semaphore
    osSemaphoreId_t longbutton_semaphore_id_ = osSemaphoreNew(1, 0, NULL);
    osThreadId_t handle_press_task_id_;
    // Button handler thread definitions
    uint32_t handle_press_task_buffer_[LONGBUTTON_THREAD_STACK_SIZE];
    StaticTask_t handle_press_task_tcb_;
    osThreadAttr_t handle_press_task_attributes_ = {
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

    // public functions
    void DisableInterrupt();
    void EnableInterrupt();
    void ClearInterrupt();

    void PollForRelease();

    

    // Peripheral information
    GPIO_TypeDef *port_;            // HAL GPIO port
    uint16_t pin_;                  // Pin number
    uint32_t debounce_time_ms_;     // Debounce time in ms
    GPIO_PinState default_state_;   // Default pin state
    bool toggle_state_;             // Toggle state of button, toggled on single and double press
    uint32_t check_release_time_;    // Periodic amount of time (in ms) before button release is checked


};
