#include "threads.h"

/* Thread definitions */
uint32_t sd_task_buffer[2048];
StaticTask_t sd_task_control_block;
const osThreadAttr_t sd_task_attributes = {
    .name = "SD Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &sd_task_control_block,
    .cb_size = sizeof(sd_task_control_block),
    .stack_mem = &sd_task_buffer[0],
    .stack_size = sizeof(sd_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t sd_task_id = osThreadNew((osThreadFunc_t)SDTask, NULL, &sd_task_attributes);

uint32_t voice_task_buffer[2048];
StaticTask_t voice_task_control_block;
const osThreadAttr_t voice_task_attributes = {
    .name = "Voice Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &voice_task_control_block,
    .cb_size = sizeof(voice_task_control_block),
    .stack_mem = &voice_task_buffer[0],
    .stack_size = sizeof(voice_task_buffer),
    .priority = (osPriority_t) osPriorityNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t voice_task_id = osThreadNew((osThreadFunc_t)VoiceTask, NULL, &voice_task_attributes);

uint32_t menu_task_buffer[2048];
StaticTask_t menu_task_control_block;
const osThreadAttr_t menu_task_attributes = {
    .name = "Menu Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &menu_task_control_block,
    .cb_size = sizeof(menu_task_control_block),
    .stack_mem = &menu_task_buffer[0],
    .stack_size = sizeof(menu_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t menu_task_id = osThreadNew((osThreadFunc_t)MenuTask, NULL, &menu_task_attributes);

uint32_t input_task_buffer[2048];
StaticTask_t input_task_control_block;
const osThreadAttr_t input_task_attributes = {
    .name = "Input Handler Task",
    .attr_bits = osThreadDetached,
    .cb_mem = &input_task_control_block,
    .cb_size = sizeof(input_task_control_block),
    .stack_mem = &input_task_buffer[0],
    .stack_size = sizeof(input_task_buffer),
    .priority = (osPriority_t) osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
osThreadId_t input_task_id = osThreadNew((osThreadFunc_t)InputHandlerTask, NULL, &input_task_attributes);

/* Periodic thread definitions */
const osTimerAttr_t joystick_timer_attr = {
    .name = "Joystick Periodic Task",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t joystick_timer_id = osTimerNew((osThreadFunc_t)JoystickTask, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &joystick_timer_attr);

/* Event flag to trigger regular task */
osEventFlagsId_t regular_event = osEventFlagsNew(NULL);
osEventFlagsId_t filename_event = osEventFlagsNew(NULL);
osEventFlagsId_t lineread_event = osEventFlagsNew(NULL);

/* Mutexes */
StaticSemaphore_t   spi_mutex_control_block;
const osMutexAttr_t spi_mutex_attributes = {
    .name = "SPI Mutex",
    .attr_bits = osMutexRecursive,
    .cb_mem = &spi_mutex_control_block,
    .cb_size = sizeof(spi_mutex_control_block),
};
osMutexId_t spi_mutex_id = osMutexNew(&spi_mutex_attributes);

StaticSemaphore_t  text_mutex_control_block;
const osMutexAttr_t text_mutex_attributes = {
    .name = "Text Mutex",
    .attr_bits = osMutexRecursive,
    .cb_mem = &text_mutex_control_block,
    .cb_size = sizeof(text_mutex_control_block),
};
osMutexId_t text_mutex_id = osMutexNew(&text_mutex_attributes);

/* Buffers */
char text_buffer[100];
char filename_buffer[20][100];

/* Thread function definitions */
void JoystickTask(void *argument) {
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

void SDTask(void *argument) {
    FIL fil;
    FRESULT fres;
    uint32_t num_lines;

    while (1) {
        // Wait for file open request (account for 1 offset so 0 produces a flag)
        uint32_t file_num = osEventFlagsWait(filename_event, 0xFFFF, osFlagsWaitAny, osWaitForever);
        file_num--;

        // Open file and read first line
        TCHAR pathname[120] = "/data/";
        strcat(pathname, filename_buffer[file_num]);
        fres = f_open(&fil, pathname, FA_READ);
        if (fres != FR_OK) {
            Logger::LogError("Error opening file: %d\n", fres);
            ui.DrawText(&FontStyle_Emulogic, "Error opening file", 30, 195, ST7789_RED);
        }
        num_lines = 0;

        char* ret;
        do {
            // Read line from file
            //osMutexAcquire(text_mutex_id, osWaitForever);
            ret = f_gets(text_buffer, sizeof(text_buffer), &fil);
            //osMutexRelease(text_mutex_id);

            // If EOF, close file and break
            if (!ret) {
                f_close(&fil);
                break;
            }

            // Signal line read
            osEventFlagsSet(lineread_event, SD_LINE_RESPONSE);

            if (num_lines == 9) {
                ui.Clear();
                num_lines = 0;
            }

            // Print line to display
            osMutexAcquire(spi_mutex_id, osWaitForever);
            ui.DrawText(&FontStyle_Emulogic, text_buffer, 30, 195-(num_lines*20), ST7789_BLACK);
            osMutexRelease(spi_mutex_id);

            // Increment num_lines
            num_lines++;

            // Wait for line read request
            uint32_t request = osEventFlagsWait(lineread_event, SD_LINE_REQUEST | SD_CLOSE_REQUEST, osFlagsWaitAny, osWaitForever);
            if (request == SD_CLOSE_REQUEST) {
                f_close(&fil);
                break;
            }
        } while (ret);

    }
}

void VoiceTask(void *argument) {
    while (1) {
        // Wait for line to appear in text buffer
        osEventFlagsWait(lineread_event, SD_LINE_RESPONSE, osFlagsWaitAny, osWaitForever);

        osMutexAcquire(text_mutex_id, osWaitForever);
        // Don't read byte
        if (text_buffer[0] != '#')
            voice.say(text_buffer);
        osMutexRelease(text_mutex_id);

        // Request next line
        osEventFlagsSet(lineread_event, SD_LINE_REQUEST);
    }
}

void MenuTask(void *argument) {
    FRESULT res;
    DIR dir;
    FILINFO fno;
    int nfile = 0, ndir = 0;

    uint32_t selected_file = 0;

    while (1) {
        // Reset counters
        nfile = 0;
        ndir = 0;
        selected_file = 0;

        osMutexAcquire(spi_mutex_id, osWaitForever);

        // Suspend voice thread and UI thread
        osThreadSuspend(voice_task_id);
        osThreadSuspend(input_task_id);

        // Display menu screen
        ui.DisplayBanner("Main Menu");
        ui.Clear();

        // Read directory items
        res = f_opendir(&dir, "data");
        if (res != FR_OK)
            Logger::LogError("Error changing directory: %d\n", res);
        // res = f_chdir("data");
        if (res == FR_OK) {
            ndir = 0;
            nfile = 0;
            while (1) {
                res = f_readdir(&dir, &fno);

                // End of directory
                if (res != FR_OK || fno.fname[0] == 0) break;   

                // Directory, ignore
                if (fno.fattrib & AM_DIR) {
                    Logger::LogInfo("Directory: %s\n", fno.fname);
                    ndir++;
                }

                // File, display on LCD
                else {
                    Logger::LogInfo("File: %s\n", fno.fname);
                    // If first file, display select icon
                    if (nfile == 0)
                        ui.DrawText(&FontStyle_Emulogic, ">", 30, 195, ST7789_RED);
                    ui.DrawText(&FontStyle_Emulogic, fno.fname, 50, 195 - (nfile*20), ST7789_BLACK);
                    ui.DrawRectangle(0, 195 - (nfile*20) - 5, 320, 1, ST7789_GRAY);

                    // Copy filename to buffer
                    strcpy(filename_buffer[nfile], fno.fname);
                    nfile++;
                }
            }
        }

        osMutexRelease(spi_mutex_id);

        // Wait for user input
        while (1) {
            uint32_t input_flag = osEventFlagsWait(regular_event, 0x1F, osFlagsWaitAny, osWaitForever);

            switch (input_flag) {
                case 0x1:                       // Joystick Up
                    // Move cursor up
                    if (selected_file > 0) {
                        osMutexAcquire(spi_mutex_id, osWaitForever);
                        MoveCursor(selected_file, selected_file - 1); 
                        selected_file--;
                        osMutexRelease(spi_mutex_id);  
                    }   
                    break;
                case 0x2:                       // Joystick Right
                case 0x4:                       // Joystick Down
                    // Move cursor down
                    if ((int)selected_file < nfile-1 && selected_file < 20) {
                        osMutexAcquire(spi_mutex_id, osWaitForever);
                        MoveCursor(selected_file, selected_file + 1); 
                        selected_file++;
                        osMutexRelease(spi_mutex_id);                   
                    }       
                    break;
                case 0x8:                       // Joystick Left
                    break;
                case 0x10:                      // Joystick Button Single Press
                    // Play selected file
                    ui.DisplayBanner(filename_buffer[selected_file]);
                    ui.Clear();
                    // Set event flag to open file, offset by 1 so 0 produces a flag
                    osEventFlagsSet(filename_event, selected_file+1);
                    osThreadResume(voice_task_id);
                    osThreadResume(input_task_id);
                    osThreadSuspend(menu_task_id);
                    break;
                default:
                    break;
            }

            if (input_flag == JOY_BTN)
                break;
        }
    }
}

void InputHandlerTask(void *argument) {
    uint32_t backlight = 50;
    uint32_t volume = 50;
    bool paused = false;

    while (1) {
        uint32_t input_flag = osEventFlagsWait(regular_event, 0x3F, osFlagsWaitAny, osWaitForever);

        switch (input_flag) {
            case 0x1:                       // Joystick Up
                Logger::LogInfo("Up\n");
                // Raise volume
                if (volume < 50) {
                    volume += 5;
                    SetVolume(volume);
                }
                break;
            case 0x2:                       // Joystick Right
                Logger::LogInfo("Right\n");
                // Raise brightness
                if (backlight < 50) {
                    backlight += 5;
                    SetBacklight(backlight);
                }
                break;
            case 0x4:                       // Joystick Down
                Logger::LogInfo("Down\n");
                // Lower volume
                if (volume > 0) {
                    volume -= 5;
                    SetVolume(volume);
                }
                break;
            case 0x8:                       // Joystick Left
                Logger::LogInfo("Left\n");
                // Lower brightness
                if (backlight > 0) {
                    backlight -= 5;
                    SetBacklight(backlight);
                }

                break;
            case 0x10:                      // Joystick Button Single Press
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
            case 0x20:                      // Joystick Button Double Press
                Logger::LogInfo("Button Double\n");

                // Suspend voice and SD thread
                osThreadSuspend(voice_task_id);
                osEventFlagsSet(lineread_event, SD_CLOSE_REQUEST);

                // On restart, clear response flag
                osEventFlagsClear(lineread_event, SD_LINE_REQUEST | SD_LINE_RESPONSE | SD_CLOSE_REQUEST);

                // Resume menu thread
                osThreadResume(menu_task_id);
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