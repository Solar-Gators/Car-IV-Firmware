#include "threads.h"

/* Setup regular threads */
osThreadId_t read_temperature_thread_id;
uint32_t read_temperature_thread_buffer[128];
StaticTask_t read_temperature_thread_control_block;
const osThreadAttr_t regular_task_attributes = {
    .name = "Read Temperature Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &read_temperature_thread_control_block,
    .cb_size = sizeof(read_temperature_thread_control_block),
    .stack_mem = &read_temperature_thread_buffer[0],
    .stack_size = sizeof(read_temperature_thread_buffer),
    .priority = (osPriority_t) osPriorityNormal,    // Lower priority than timer
    .tz_module = 0,
    .reserved = 0,
};

/* Setup periodic threads */
osTimerAttr_t voltage_periodic_timer_attr = {
    .name = "Read Voltage Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t voltage_timer_id = osTimerNew((osThreadFunc_t)ReadVoltageThread, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &voltage_periodic_timer_attr);

osTimerAttr_t current_periodic_timer_attr = {
    .name = "Read Current Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t current_timer_id = osTimerNew((osThreadFunc_t)ReadCurrentThread, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &current_periodic_timer_attr);

osTimerAttr_t temperature_periodic_timer_attr = {
    .name = "Read Temperature Periodic",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t temperature_timer_id = osTimerNew((osThreadFunc_t)ReadTemperatureThread, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &temperature_periodic_timer_attr);

osTimerAttr_t broadcast_periodic_timer_attr = {
    .name = "Broadcast Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t broadcast_timer_id = osTimerNew((osThreadFunc_t)ReadTemperatureThread, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &broadcast_periodic_timer_attr);

/* Mutexes */
StaticSemaphore_t adc_mutex_cb;
const osMutexAttr_t adc_mutex_attr = {
    .name = "ADC Mutex",
    .attr_bits = osMutexRecursive | osMutexPrioInherit | osMutexRobust,
    .cb_mem = &adc_mutex_cb,
    .cb_size = sizeof(adc_mutex_cb),
};
osMutexId_t adc_mutex_id = osMutexNew(NULL);

/* Event flag to trigger read_temperature_thread */
osEventFlagsId_t read_temperature_event = osEventFlagsNew(NULL);

/* Start periodic threads */
void ThreadsStart() {
    // Toggle read voltage thread every 25ms (40Hz)
    osTimerStart(voltage_timer_id, 25);

    // Toggle read current thread every 8ms (125Hz)
    osTimerStart(current_timer_id, 8);

    // Toggle read temperature thread every 1000ms (1Hz)
    osTimerStart(temperature_timer_id, 1000);

    // Broadcast BMS frames every 2500ms (0.4Hz)
    osTimerStart(broadcast_timer_id, 2500);
}

/* 
 * Periodic thread function to read the voltage of each cell in the battery
 */
void ReadVoltageThread(void *argument) {
    bms.ReadVoltages();

    for (int i = 0; i < 20; i++) {
        Logger::LogInfo("Cell %d: %d", i, bms.GetCellVoltage(i));
    }

    // Update the BMS frame
    BMSFrame0::Instance().SetPackVoltage(bms.GetPackVoltage());
    BMSFrame0::Instance().SetAvgCellVoltage(bms.GetAvgCellVoltage());
    BMSFrame0::Instance().SetHighCellVoltage(bms.GetHighCellVoltage());
    BMSFrame0::Instance().SetLowCellVoltage(bms.GetLowCellVoltage());

    // Log the voltages every 2.5 seconds
    static int counter = 0;
    if (counter++ % 100 == 0) {
        // Logger::LogInfo("Pack Voltage: %d", bms.GetPackVoltage());
        // Logger::LogInfo("Avg Cell Voltage: %d", bms.GetAvgCellVoltage());
        // Logger::LogInfo("High Cell Voltage: %d", bms.GetHighCellVoltage());
        // Logger::LogInfo("Low Cell Voltage: %d", bms.GetLowCellVoltage());

        // TODO: Debug only, remove
        HAL_GPIO_TogglePin(CONTACTOR_SOURCE_SEL_GPIO_Port, CONTACTOR_SOURCE_SEL_Pin);
    }
}

/* 
 * Periodic thread function to read the current of each cell in the battery
 */
void ReadCurrentThread(void *argument) {
    // Read current H and L from adc0
    uint16_t current[2];

    HAL_GPIO_WritePin(CURRENT_EN_GPIO_Port, CURRENT_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_SET);
    osDelay(1);

    osMutexAcquire(adc_mutex_id, osWaitForever);
    adcs[0].ConversionReadAutoSequence(current, 2);
    osMutexRelease(adc_mutex_id);

    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURRENT_EN_GPIO_Port, CURRENT_EN_Pin, GPIO_PIN_RESET);

    // TODO: Interpret current values
}

/*
 * Periodic thread to trigger start of thermistor readings
 */
void ReadTemperaturePeriodic(void *argument) {
    osEventFlagsSet(read_temperature_event, 0x1);
}

/* 
 * Periodic thread function to read the temperature of each cell in the battery
 */
void ReadTemperatureThread(void *argument) {
    SetAmplifierState(true);

    // TODO: Figure out a reasonable delay to allow values to settle
    osDelay(5);

    // Local array to store raw thermistor values
    // Array is 1 indexed, 0 is reserved
    uint16_t thermistor_vals[23];

    // For adc0, manually read channels except 5 and 7
    // TODO: This is buggy, for some reason adc0 updates thermistor 5 with every read
    // Read the returned channel value from the thermistor to figure out what's going on
    uint8_t adc0_thermistor_channels[6] = {0, 1, 2, 3, 4, 6};
    for (auto channel : adc0_thermistor_channels) {
        // Acquire mutex every time to avoid blocking current sensing for too long
        osMutexAcquire(adc_mutex_id, osWaitForever);

        adcs[0].ManualSelectChannel(channel);
        adcs[0].ConversionReadManual(&thermistor_vals[MapADCChannelToThermistor(0, channel)], 
                                    channel);

        osMutexRelease(adc_mutex_id);
    }

    // For adc1, read auto-sequence all channels
    uint16_t temp_thermistor_vals[8];
    osMutexAcquire(adc_mutex_id, osWaitForever);
    adcs[1].ConversionReadAutoSequence(temp_thermistor_vals, 8);
    osMutexRelease(adc_mutex_id);
    for (int i = 0; i < 8; i++)
        thermistor_vals[MapADCChannelToThermistor(1, i)] = temp_thermistor_vals[i];

    // For adc2, read auto-sequence all channels
    osMutexAcquire(adc_mutex_id, osWaitForever);
    adcs[2].ConversionReadAutoSequence(temp_thermistor_vals, 8);
    osMutexRelease(adc_mutex_id);
    for (int i = 0; i < 8; i++)
        thermistor_vals[MapADCChannelToThermistor(2, i)] = temp_thermistor_vals[i];

    // Test auto-sequence
    // uint16_t thermistor_vals[8];
    // adcs[2].ConversionReadAutoSequence(thermistor_vals, 8);

    // for (int i = 0; i < 8; i++) {
    //     float voltage = (float)thermistor_vals[i] * 3.3 / 0x10000;
    //     char char_buf[50];
    //     sprintf(char_buf, "Voltage %d: %f", i, voltage);
    //     Logger::LogInfo(char_buf);
    // }

    // Output voltage values for all channels
    // TODO: Debug only, remove later
    for (int i = 1; i <= 22; i++) {
        float temperature = ADCToTemp(thermistor_vals[i]);
        char char_buf[50];
        // TODO: Use etl formatting instead of sprintf
        sprintf(char_buf, "Thermistor %d: %f", i, temperature);
        //Logger::LogInfo(char_buf);
    }

    // Find minimum and maximum cell temp and ids
    uint16_t max_temp = 0;
    uint16_t min_temp = 0xFFFF;
    uint8_t max_temp_id, min_temp_id = 0;

    for (int i = 1; i <= 20; i++) {
        if (thermistor_vals[i] > max_temp) {
            max_temp = thermistor_vals[i];
            max_temp_id = i;
        }

        if (thermistor_vals[i] < min_temp) {
            min_temp = thermistor_vals[i];
            min_temp_id = i;
        }
    }

    // Populate datamodule
    BMSFrame1::Instance().SetHighTemp(max_temp);
    BMSFrame1::Instance().SetHighTempCellID(max_temp_id);
    BMSFrame1::Instance().SetLowTemp(min_temp);
    BMSFrame1::Instance().SetLowTempCellID(min_temp_id);

    SetAmplifierState(false);
}

void BroadcastThread(void *argument) {
    // Broadcast BMS frames
    CANController::Send(&BMSFrame0::Instance());
    CANController::Send(&BMSFrame1::Instance());
    CANController::Send(&BMSFrame2::Instance());
}