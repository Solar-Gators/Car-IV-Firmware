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
static const uint32_t read_voltage_period = 25;
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

static const uint32_t read_current_period = 50;
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

static const uint32_t read_temperature_period = 1000;
osTimerAttr_t temperature_periodic_timer_attr = {
    .name = "Read Temperature Periodic",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
// osTimerId_t temperature_timer_id = osTimerNew((osThreadFunc_t)ReadTemperatureThread, 
//                                             osTimerPeriodic, 
//                                             NULL, 
//                                             &temperature_periodic_timer_attr);

static const uint32_t broadcast_period = 2500;
osTimerAttr_t broadcast_periodic_timer_attr = {
    .name = "Broadcast Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t broadcast_timer_id = osTimerNew((osThreadFunc_t)ReadTemperaturePeriodic, 
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
    osTimerStart(voltage_timer_id, read_voltage_period);

    // Toggle read current thread every 8ms (125Hz)
    osTimerStart(current_timer_id, read_current_period);

    // Toggle read temperature thread every 1000ms (1Hz)
    osTimerStart(temperature_timer_id, read_temperature_period);

    // Broadcast BMS frames every 2500ms (0.4Hz)
    osTimerStart(broadcast_timer_id, broadcast_period);
}

/* 
 * Periodic thread function to read the voltage of each cell in the battery
 */
void ReadVoltageThread(void *argument) {
    static uint8_t high_cell_voltage_id = 0;
    static int16_t high_cell_voltage = bms_config.MIN_CELL_VOLTAGE;
    static uint8_t low_cell_voltage_id = 0;
    static int16_t low_cell_voltage = bms_config.MAX_CELL_VOLTAGE;
    static int16_t sum_cell_voltage = 0;
    static int16_t avg_cell_voltage = 0;

    bms.ReadVoltages();

    for (int i = 0; i < bms_config.NUM_CELLS; i++) {
        int16_t cell_voltage = bms.GetCellVoltage(i);

        if (cell_voltage > high_cell_voltage) {
            high_cell_voltage = cell_voltage;
            high_cell_voltage_id = i;
        }

        if (cell_voltage < low_cell_voltage) {
            low_cell_voltage = cell_voltage;
            low_cell_voltage_id = i;
        }

        sum_cell_voltage += cell_voltage;
    }

    avg_cell_voltage = sum_cell_voltage / bms_config.NUM_CELLS;

    // Update the BMS frame
    BMSFrame0::Instance().SetPackVoltage(bms.GetPackVoltage()); // Read from pack pin
    BMSFrame0::Instance().SetAvgCellVoltage(avg_cell_voltage);
    BMSFrame0::Instance().SetHighCellVoltage(high_cell_voltage);
    BMSFrame0::Instance().SetLowCellVoltage(low_cell_voltage);

    // Check for overvoltage and undervoltage conditions
    if (high_cell_voltage > bms_config.MAX_CELL_VOLTAGE) {
        Logger::LogError("High cell voltage on cell %d: %dmV", high_cell_voltage_id, high_cell_voltage);

        // Set high cell voltage error bit
        // This error can only be cleared by a power cycle
        BMSFrame3::Instance().Data()[2] |= 0x1 << 6;
    }

    if (low_cell_voltage < bms_config.MIN_CELL_VOLTAGE) {
        Logger::LogError("Low cell voltage on cell %d: %dmV", low_cell_voltage_id, low_cell_voltage);

        // Set low cell voltage error bit
        // This error can only be cleared by a power cycle
        BMSFrame3::Instance().Data()[2] |= 0x1 << 3;
    }

    // Log the voltages every 2.5 seconds
    if (bms_config.LOG_VOLTAGE) {
        static int counter = 0;
        static constexpr int log_interval = 2500 / read_voltage_period;
        if (counter++ % log_interval == 0) {
            for (int i = 0; i < bms_config.NUM_CELLS; i++) {
                Logger::LogInfo("Cell %d Voltage: %d", i, bms.GetCellVoltage(i));
            }
            Logger::LogInfo("Pack Voltage: %d", bms.GetPackVoltage());
            Logger::LogInfo("Average Cell Voltage: %d", avg_cell_voltage);
            Logger::LogInfo("High Cell Voltage: %d on cell %d", high_cell_voltage, high_cell_voltage_id);
            Logger::LogInfo("Low Cell Voltage: %d on cell %d", low_cell_voltage, low_cell_voltage_id);
        }
    }
}

/* 
 * Periodic thread function to read the current of each cell in the battery
 */
void ReadCurrentThread(void *argument) {
    // Read current H and L from adc0
    // Current_L in adc_vals[0] and Current_H in adc_vals[1]
    static uint16_t adc_vals[2];

    HAL_GPIO_WritePin(CURRENT_EN_GPIO_Port, CURRENT_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_SET);

    // TODO: Find minimum delay to allow values to settle
    osDelay(5);

    osMutexAcquire(adc_mutex_id, osWaitForever);
    adcs[0].ConversionReadAutoSequence(&adc_vals[0], 2);
    osMutexRelease(adc_mutex_id);

    HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURRENT_EN_GPIO_Port, CURRENT_EN_Pin, GPIO_PIN_RESET);

    // Convert ADC values to current
    float current_l = ADCToCurrentL(adc_vals[0]);
    float current_h = ADCToCurrentH(adc_vals[1]);

    // If current is <50A, use current_l, else use current_h
    float current = current_l < 50.0 ? current_l : current_h;

    // Check if current is above threshold
    if (current > bms_config.MAX_DISCHARGE_CURRENT) {
        Logger::LogError("Current discharge limit exceeded: %fA", current);

        // Set current error bit
        // This error can only be cleared by a power cycle
        // TODO: Figure out fault flags
        BMSFrame3::Instance().Data()[2] |= 0x1 << 0;
    }


    if (current_l < -bms_config.MAX_CHARGE_CURRENT) {
        Logger::LogError("Current charge limit exceeded: %fA", current_l);

        // Set current error bit
        // This error can only be cleared by a power cycle
        // TODO: Figure out fault flags
        BMSFrame3::Instance().Data()[2] |= 0x1 << 0;
    }

    // Log current every 2.5 seconds
    if (bms_config.LOG_CURRENT) {
        static int counter = 0;
        static constexpr int log_interval = 2500 / read_current_period;
        if (counter++ % log_interval == 0) {
            Logger::LogInfo("Current L: %fA", current_l);
            Logger::LogInfo("Current H: %fA", current_h);
        }
    }
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
    // Local array to store raw thermistor values
    // Array is 1 indexed, 0 is reserved
    static uint16_t thermistor_vals[23];
    float temps[23];
    int min_index = 1;
    int max_index = 1;

    while (1) {
        osEventFlagsWait(read_temperature_event, 0x1, osFlagsWaitAny, osWaitForever);
        
        // Enable thermistor amplifiers
        SetAmplifierState(true);
        osDelay(5);

        // For adc0, manually read channels except 5 and 7
        // Acquire and release mutex every time to avoid blocking current sensing for too long
        uint8_t adc0_thermistor_channels[6] = {0, 1, 2, 3, 4, 6};
        for (auto channel : adc0_thermistor_channels) {
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

        // Disable thermistor amplifiers
        SetAmplifierState(false);

        // Convert raw ADC values to temperature and find min and max temp
        for (int i = 1; i <= 22; i++) {
            temps[i] = ADCToTemp(thermistor_vals[i]);

            if (thermistor_vals[i] > thermistor_vals[max_index]) {
                max_index = i;

            if (thermistor_vals[i] < thermistor_vals[min_index]) {
                min_index = i;
            }
        }

        // Log the temperatures every 2.5 seconds
        if (bms_config.LOG_TEMPERATURE) {
            static int counter = 0;
            static constexpr int log_interval = 2500 / read_temperature_period;
            if (counter++ % log_interval == 0) {
                for (int i = 1; i <= 22; i++) {
                    Logger::LogInfo("Thermistor %d: %f", i, temps[i]);
                }
            }
            Logger::LogInfo("Max Temp: %f on thermistor %d", temps[max_index], max_index);
            Logger::LogInfo("Min Temp: %f on thermistor %d", temps[min_index], min_index);
        }
    }
}

void BroadcastThread(void *argument) {
    // Broadcast BMS frames
    CANController::Send(&BMSFrame0::Instance());
    CANController::Send(&BMSFrame1::Instance());
    CANController::Send(&BMSFrame2::Instance());
}