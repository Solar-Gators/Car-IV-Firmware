#include "threads.h"

#include "etl/string.h"
#include "etl/to_string.h"
#include "etl/format_spec.h"

/* Program Summary
 * Periodic Threads:
 *  - ReadVoltageThread (40Hz): Reads the voltage of each cell in series
 *     1. Checks for overvoltage and undervoltage conditions from BMSSecondaryFrame0, triggers ErrorThread if necessary
 *     2. Update cell_voltages in local bms
 *     3. Checks for overvoltage and undervoltage conditions, triggers ErrorThread if necessary
 *     4. Updates high_cell_voltage, low_cell_voltage, avg_cell_voltage global data
 *  - ReadCurrentThread (125Hz): Reads the total pack current
 *     1. Reads current from current sensor and updates BMS frame
 *     2. Checks for overcurrent conditions, triggers ErrorThread if necessary
 *     3. Updates running total current integral
 *     4. Updates average power for current period
 *  - ReadTemperaturePeriodic (1Hz): Triggers the start of the ReadTemperatureThread
 *  - BroadcastPeriodic (10Hz): Triggers the start of the BroadcastThread
 * Regular Threads:
 *  - ReadTemperatureThread: Reads the temperature of each thermistor
 *     1. Scans all thermistor channels on all ADCs
 *     2. Checks for overtemperature conditions, triggers ErrorThread if necessary
 *  - BroadcastThread: Broadcasts BMS CAN frames
 *     1. Updates data in BMSFrame0, BMSFrame1, BMSFrame2
 *      - Updates data in BMSVoltageFrame, BMSCurrentFrame, BMSFlagsFrame
 *      - Sends the frames using CANController
 *  - ErrorThread: Triggered in case of error, takes appropriate action
 *      - Asserts error flag, only cleared by power cycle or manual reset
 *      - If overvoltage, undervoltage, overcurrent, or overtemperature,
 *          1. Opens contactors
 *          2. Updates BMSFlagsFrame with error flags
 *          3. Sends BMSFlagsFrame
 *  - ContactorsThread: Controls the contactors based on VCU frame
 * CAN Callbacks:
 *  - SecondaryFrame0Callback: Updates total pack voltage and average cell voltage
 *      1. Updates total pack, average, high, and low cell voltages with data from secondary BMS
 *      2. Checks for overvoltage and undervoltage conditions, triggers ErrorThread if necessary
 *  - SecondaryFrame1Callback: Updates temperature data from secondary BMS
 *      1. Update average, low, low ID, high, high ID temperatures with data from secondary BMS
 *      2. Checks for overtemperature conditions, triggers ErrorThread if necessary
 *  - SecondaryFrame2Callback: Updates error data from secondary BMS
 *      1. If error in secondary BMS, trigger ErrorThread
 *  - VCUFrameCallback: 
 *      1. If kill switch is not pressed and no bms errors, close contactors
 *      2. If kill switch is pressed, open contactors
 * Notes:
 *  - Voltage and current measurements are time-critical so they are read in the periodic threads.
 *     Temperature is read in a separate thread to avoid blocking the current sensing for too long.
*/

/* Global variables */
static const uint8_t num_total_cells = bms_config.NUM_CELLS_PRIMARY + bms_config.NUM_CELLS_SECONDARY;

static unsigned int voltage_thread_counter = 0;
static int16_t cell_voltages[32];  // Array to store cell voltages in mV
static int16_t high_cell_voltage = INT16_MIN;
static int16_t low_cell_voltage = INT16_MAX;
static uint8_t high_cell_voltage_id = 0;
static uint8_t low_cell_voltage_id = 0;
static uint16_t avg_cell_voltage;

static uint16_t high_temp;
static uint16_t low_temp;
static uint8_t high_temp_id;
static uint8_t low_temp_id;
static uint16_t internal_temp;

static constexpr etl::format_spec format_float(10, 5, 2, false, false, false, false, ' ');
static constexpr etl::format_spec format_int(10, 4, 0, false, false, false, false, ' ');

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

static const uint32_t read_temperature_period = 1000;
osTimerAttr_t temperature_periodic_timer_attr = {
    .name = "Read Temperature Periodic",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t temperature_timer_id = osTimerNew((osThreadFunc_t)ReadTemperaturePeriodic, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &temperature_periodic_timer_attr);

static const uint32_t broadcast_period = 1000;
osTimerAttr_t broadcast_periodic_timer_attr = {
    .name = "Broadcast Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t broadcast_timer_id = osTimerNew((osThreadFunc_t)BroadcastPeriodic, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &broadcast_periodic_timer_attr);

/* Setup regular threads */
osThreadId_t thermistor_thread_id;
uint32_t thermistor_thread_buffer[512];
StaticTask_t thermistor_thread_control_block;
const osThreadAttr_t thermistor_thread_attributes = {
    .name = "Thermistor Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &thermistor_thread_control_block,
    .cb_size = sizeof(thermistor_thread_control_block),
    .stack_mem = &thermistor_thread_buffer[0],
    .stack_size = sizeof(thermistor_thread_buffer),
    .priority = (osPriority_t) osPriorityNormal,
    .tz_module = 0,
    .reserved = 0,
};

/* Mutexes */
StaticSemaphore_t adc_mutex_cb;
const osMutexAttr_t adc_mutex_attr = {
    .name = "ADC Mutex",
    .attr_bits = osMutexRecursive | osMutexPrioInherit | osMutexRobust,
    .cb_mem = &adc_mutex_cb,
    .cb_size = sizeof(adc_mutex_cb),
};
osMutexId_t adc_mutex_id = osMutexNew(NULL);

StaticSemaphore_t logger_mutex_cb;
const osMutexAttr_t logger_mutex_attr = {
    .name = "Logger Mutex",
    .attr_bits = osMutexRecursive | osMutexPrioInherit | osMutexRobust,
    .cb_mem = &logger_mutex_cb,
    .cb_size = sizeof(logger_mutex_cb),
};
osMutexId_t logger_mutex_id = osMutexNew(NULL);

/* Event flag to trigger read_thermistor_thread */
osEventFlagsId_t read_temperature_event = osEventFlagsNew(NULL);

/* Event flag to trigger CAN broadcast */
osEventFlagsId_t broadcast_event = osEventFlagsNew(NULL);

/* Event flag to trigger error handler */
osEventFlagsId_t error_event = osEventFlagsNew(NULL);

/* Start periodic threads */
void ThreadsStart() {
    // Toggle read voltage thread every 25ms (40Hz)
    osTimerStart(voltage_timer_id, read_voltage_period);

    // Toggle read temperature thread every 1000ms (1Hz)
    osTimerStart(temperature_timer_id, read_temperature_period);

    // Broadcast BMS frames every 2500ms (0.4Hz)
    osTimerStart(broadcast_timer_id, broadcast_period);

    // osEventFlagsSet(error_event, 0x1); // Trigger contactors once

    // Initialize regular threads
    thermistor_thread_id = osThreadNew((osThreadFunc_t)ReadTemperatureThread, NULL, &thermistor_thread_attributes);
}

/* 
 * Periodic thread function to read the voltage of each cell in the battery
 */
void ReadVoltageThread(void *argument) {
    voltage_thread_counter++;

    high_cell_voltage = INT16_MIN;
    low_cell_voltage = INT16_MAX;
    high_cell_voltage_id = 0;
    low_cell_voltage_id = 0;
    uint32_t sum_cell_voltage = 0;

    // Update cell voltage values in bms
    bms.ReadVoltages();

    for (int i = 0; i < bms_config.NUM_CELLS_SECONDARY; i++) {
        // Populate cell_voltages array
        // Note: This step is unecessary, all the voltage values are stored in the bms
        // driver already. If getting values from secondary bms, putting everything in
        // one array keeps data consistent.
        cell_voltages[i] = bms.GetCellVoltage(i);

        // Update high cell voltage
        if (cell_voltages[i] > high_cell_voltage) {
            high_cell_voltage = cell_voltages[i];
            high_cell_voltage_id = i;
        }

        // Update low cell voltage
        if (cell_voltages[i] < low_cell_voltage) {
            low_cell_voltage = cell_voltages[i];
            low_cell_voltage_id = i;
        }

        // Update sum of cell voltages for computing average
        sum_cell_voltage += cell_voltages[i];
    }

    // Compute average cell voltage
    avg_cell_voltage = sum_cell_voltage / bms_config.NUM_CELLS_SECONDARY;

    // Check for overvoltage and undervoltage conditions on local bms
    if (high_cell_voltage > bms_config.MAX_CELL_VOLTAGE) {
        Logger::LogError("High cell voltage on cell %d: %dmV", high_cell_voltage_id, high_cell_voltage);

        // Set high cell voltage error bit
        // This error can only be cleared by a power cycle
    }

    if (low_cell_voltage < bms_config.MIN_CELL_VOLTAGE) {
        Logger::LogError("Low cell voltage on cell %d: %dmV", low_cell_voltage_id, low_cell_voltage);

        // Set low cell voltage error bit
        // This error can only be cleared by a power cycle
    }

    // Populate voltage CANFrames
    BMSSecondaryFrame0::Instance().SetSubpackVoltage(sum_cell_voltage / 10); // Units in 0.01V
    BMSSecondaryFrame0::Instance().SetAverageVoltage(avg_cell_voltage);
    BMSSecondaryFrame0::Instance().SetHighVoltage(high_cell_voltage);
    BMSSecondaryFrame0::Instance().SetLowVoltage(low_cell_voltage);
    BMSSecondaryFrame1::Instance().SetHighCellVoltageID(high_cell_voltage_id);
    BMSSecondaryFrame1::Instance().SetLowCellVoltageID(low_cell_voltage_id);
    BMSSecondaryFrame1::Instance().SetHighCellVoltageID(high_cell_voltage_id);
    BMSSecondaryFrame1::Instance().SetLowCellVoltageID(low_cell_voltage_id);
    CANController::Send(&BMSSecondaryFrame0::Instance());
    CANController::Send(&BMSSecondaryFrame1::Instance());

    // If configured, log the voltages every 2.5 seconds
    if (bms_config.LOG_VOLTAGE) {
        static int counter = 0;
        static constexpr int log_interval = 2500 / read_voltage_period;
        if (counter++ % log_interval == 0) {
            for (int i = 0; i < bms_config.NUM_CELLS_PRIMARY + bms_config.NUM_CELLS_SECONDARY; i++) {
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
        for (int i = 0; i < 8; i++)
            thermistor_vals[MapADCChannelToThermistor(2, i)] = temp_thermistor_vals[i];

        // Disable thermistor amplifiers
        SetAmplifierState(false);
        osMutexRelease(adc_mutex_id);

        // Convert raw ADC values to temperature and find min and max temp
        for (int i = 1; i <= bms_config.NUM_THERMISTORS_SECONDARY; i++) {
            temps[i] = ADCToTemp(thermistor_vals[i]);

            // Thermistor unpopulated (or its just really cold)
            bool thermistor_unpopulated = false;
            if (temps[i] < 16.5)
                thermistor_unpopulated = true;
            if (thermistor_unpopulated) {
                Logger::LogWarning("Thermistor %d unpopulated", i);
                uint8_t status = BMSSecondaryFrame3::Instance().GetStatusFlags();
                status |= 0x1;
                BMSSecondaryFrame3::Instance().SetStatusFlags(status);
                CANController::Send(&BMSSecondaryFrame3::Instance());
            }
            // Thermistor is here
            else {
                if (thermistor_vals[i] > thermistor_vals[max_index])
                max_index = i;

                if (thermistor_vals[i] < thermistor_vals[min_index])
                    min_index = i;
            }        
        }

        // Get high and low temps
        high_temp = temps[max_index] * 100;
        low_temp = temps[min_index] * 100;
        high_temp_id = max_index + bms_config.NUM_CELLS_PRIMARY;
        low_temp_id = min_index + bms_config.NUM_CELLS_PRIMARY;

        // Get internal temp
        internal_temp = ADCToTemp(thermistor_vals[22]) * 100;

        // Capture temperature data
        BMSSecondaryFrame2::Instance().SetHighTemp(high_temp);
        BMSSecondaryFrame2::Instance().SetLowTemp(low_temp);
        BMSSecondaryFrame2::Instance().SetHighTempCellID(high_temp_id);
        BMSSecondaryFrame2::Instance().SetLowTempCellID(low_temp_id);
        BMSSecondaryFrame2::Instance().SetInternalTemp(internal_temp);
        CANController::Send(&BMSSecondaryFrame2::Instance());

        // Log the temperatures every 2.5 seconds if configured
        if (bms_config.LOG_TEMPERATURE) {
            static int counter = 0;
            static constexpr int log_interval = 2500 / read_temperature_period;
            if (counter++ % log_interval == 0) {
                etl::string<5> float_buf;
                osMutexAcquire(logger_mutex_id, osWaitForever);
                for (int i = 1; i <= 22; i++) {
                    etl::to_string(temps[i], float_buf, format_float, false);
                    Logger::LogInfo("Thermistor %d temp: %s", i, float_buf.c_str());
                }
                etl::to_string(temps[max_index], float_buf, format_float, false);
                Logger::LogInfo("Max Temp: %s on thermistor %d", float_buf.c_str(), max_index);
                etl::to_string(temps[min_index], float_buf, format_float, false);
                Logger::LogInfo("Min Temp: %s on thermistor %d", float_buf.c_str(), min_index);
                osMutexRelease(logger_mutex_id);
            }
        }
    }
}

void BroadcastPeriodic(void *argument) {
    // Broadcast status and number of cells
    // Information in frame is updated elsewhere
    CANController::Send(&BMSFrame3::Instance());
}