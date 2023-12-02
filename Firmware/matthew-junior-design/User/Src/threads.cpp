#include "threads.h"

/* Thread definitions */
uint32_t voice_task_buffer[128];
StaticTask_t voice_task_control_block;
const osThreadAttr_t voice_task_attributes = {
    .name = "Voice Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &voice_task_control_block,
    .cb_size = sizeof(voice_task_control_block),
    .stack_mem = &voice_task_buffer[0],
    .stack_size = sizeof(voice_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t voice_task_id = osThreadNew((osThreadFunc_t)VoiceTask, NULL, &voice_task_attributes);

uint32_t ui_task_buffer[128];
StaticTask_t ui_task_control_block;
const osThreadAttr_t ui_task_attributes = {
    .name = "UI Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &ui_task_control_block,
    .cb_size = sizeof(ui_task_control_block),
    .stack_mem = &ui_task_buffer[0],
    .stack_size = sizeof(ui_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t ui_task_id = osThreadNew((osThreadFunc_t)UITask, NULL, &ui_task_attributes);

/* Periodic thread definitions */
const osTimerAttr_t periodic_timer_attr = {
    .name = "Periodic Task 1",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t periodic_timer_id = osTimerNew((osThreadFunc_t)PeriodicTask1, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &periodic_timer_attr);

/* Event flag to trigger regular task */
osEventFlagsId_t regular_event = osEventFlagsNew(NULL);


/* Thread function definitions */
void PeriodicTask1(void *argument) {
    uint16_t joy_val = GetJoyXY();

    uint8_t joy_x = joy_val >> 8;
    uint8_t joy_y = joy_val & 0xFF;

    // Center joystick values around 0
    int32_t x_pos = joy_x - 100;
    int32_t y_pos = joy_y - 110;

    uint32_t x_abs = x_pos > 0 ? x_pos : -x_pos;
    uint32_t y_abs = y_pos > 0 ? y_pos : -y_pos;

    // Get most extreme value that is out of deadzone
    if (x_abs > y_abs && x_abs > 20) {
        // Right
        if (x_pos > 0) {
            osEventFlagsSet(regular_event, 0x2);
        }
        // Left
        else {
            osEventFlagsSet(regular_event, 0x8);
        }
    }
    else if (y_abs > x_abs && y_abs > 20) {
        // Down
        if (y_pos > 0) {
            osEventFlagsSet(regular_event, 0x4);
        }
        // Up
        else {
            osEventFlagsSet(regular_event, 0x1);
        }
    }
}

void VoiceTask(void *argument) {
    while (1) {
        display.Fill(ST7789_WHITE);
        display.DrawText(&FontStyle_Emulogic, "How happy is the little stone", 30, 200, ST7789_BLACK);
        voice.say("How happy is the little stone,");
        display.DrawText(&FontStyle_Emulogic, "That rambles in the road alone,", 30, 185, ST7789_BLACK);
        voice.say("That rambles in the road alone,");
        display.DrawText(&FontStyle_Emulogic, "And doesn't care about careers,", 30, 170, ST7789_BLACK);
        voice.say("And doesn't care about careers,");
        display.DrawText(&FontStyle_Emulogic, "And exigencies never fears;", 30, 155, ST7789_BLACK);
        voice.say("And exigencies never fears;");
        display.DrawText(&FontStyle_Emulogic, "Whose coat of elemental brown", 30, 140, ST7789_BLACK);
        voice.say("Whose coat of elemental brown,");
        display.DrawText(&FontStyle_Emulogic, "A passing universe put on;", 30, 125, ST7789_BLACK);
        voice.say("A passing universe put on;");
        display.DrawText(&FontStyle_Emulogic, "And independent as the sun,", 30, 110, ST7789_BLACK);
        voice.say("And independent as the sun,");
        display.DrawText(&FontStyle_Emulogic, "Associates or grows alone,", 30, 95, ST7789_BLACK);
        voice.say("Associates or grows alone,");
        display.DrawText(&FontStyle_Emulogic, "Fulfilling absolute decree", 30, 80, ST7789_BLACK);
        voice.say("Fulfilling absolute decree,,");
        display.DrawText(&FontStyle_Emulogic, "In casual simplicity", 30, 65, ST7789_BLACK);
        voice.say("In casual simplicity");
    }
}

void UITask(void *argument) {
    uint32_t backlight = 50;
    bool paused = false;

    while (1) {
        uint32_t input_flag = osEventFlagsWait(regular_event, 0x1F, osFlagsWaitAny, osWaitForever);

        switch (input_flag) {
            case 0x1:
                Logger::LogInfo("Up\n");
                break;
            case 0x2:
                Logger::LogInfo("Right\n");
                // Raise brightness
                if (backlight < 50) {
                    backlight += 5;
                    SetBacklight(backlight);
                }

                break;
            case 0x4:
                Logger::LogInfo("Down\n");
                break;
            case 0x8:
                Logger::LogInfo("Left\n");
                // Lower brightness
                if (backlight > 0) {
                    backlight -= 5;
                    SetBacklight(backlight);
                }

                break;
            case 0x10:
                Logger::LogInfo("Button\n");

                // Play/pause voice thread
                if (!paused) {
                    paused = true;
                    osThreadSuspend(voice_task_id);
                }
                else {
                    paused = false;
                    osThreadResume(voice_task_id);
                }


                break;
            default:
                break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == JOY_BTN_Pin) {
        // Start debounce timer
        htim4.Instance->CNT = 0;
        HAL_TIM_Base_Start_IT(&htim4);
    }
}