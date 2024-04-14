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

static uint8_t fault_flags = 0;
static uint8_t status_flags = 0;

static unsigned int voltage_thread_counter = 0;
static uint16_t cell_voltages[32];  // Array to store cell voltages in mV
static uint32_t local_pack_voltage; // Voltage of pack voltage measured by this BMS in mV
static uint32_t total_pack_voltage; // Voltage of pack voltage measured by all BMS in mV
static uint16_t high_cell_voltage = bms_config.MIN_CELL_VOLTAGE;
static uint16_t low_cell_voltage = bms_config.MAX_CELL_VOLTAGE;
static uint8_t high_cell_voltage_id = 0;
static uint8_t low_cell_voltage_id = 0;
static uint16_t avg_cell_voltage;

static unsigned int current_thread_counter = 0;
static float pack_current;
static uint16_t integral_current;

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
osTimerId_t temperature_timer_id = osTimerNew((osThreadFunc_t)ReadTemperaturePeriodic, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &temperature_periodic_timer_attr);

static const uint32_t broadcast_period = 100;
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

osThreadId_t broadcast_thread_id;
uint32_t broadcast_thread_buffer[128];
StaticTask_t broadcast_thread_control_block;
const osThreadAttr_t broadcast_thread_attributes = {
    .name = "Broadcast Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &broadcast_thread_control_block,
    .cb_size = sizeof(broadcast_thread_control_block),
    .stack_mem = &broadcast_thread_buffer[0],
    .stack_size = sizeof(broadcast_thread_buffer),
    .priority = (osPriority_t) osPriorityNormal,
    .tz_module = 0,
    .reserved = 0,
};

osThreadId_t error_thread_id;
uint32_t error_thread_buffer[64];
StaticTask_t error_thread_control_block;
const osThreadAttr_t error_thread_attributes = {
    .name = "Error Thread",
    .attr_bits = osThreadDetached,
    .cb_mem = &error_thread_control_block,
    .cb_size = sizeof(error_thread_control_block),
    .stack_mem = &error_thread_buffer[0],
    .stack_size = sizeof(error_thread_buffer),
    .priority = (osPriority_t) osPriorityHigh,
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

StaticSemaphore_t current_integral_mutex_cb;
const osMutexAttr_t current_integral_mutex_attr = {
    .name = "Current Integral Mutex",
    .attr_bits = osMutexRecursive | osMutexPrioInherit | osMutexRobust,
    .cb_mem = &current_integral_mutex_cb,
    .cb_size = sizeof(current_integral_mutex_cb),
};
osMutexId_t current_integral_mutex_id = osMutexNew(NULL);

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

    // Toggle read current thread every 8ms (125Hz)
    osTimerStart(current_timer_id, read_current_period);

    // Toggle read temperature thread every 1000ms (1Hz)
    osTimerStart(temperature_timer_id, read_temperature_period);

    // Broadcast BMS frames every 2500ms (0.4Hz)
    osTimerStart(broadcast_timer_id, broadcast_period);

    // Initialize regular threads
    thermistor_thread_id = osThreadNew((osThreadFunc_t)ReadTemperatureThread, NULL, &thermistor_thread_attributes);
    broadcast_thread_id = osThreadNew((osThreadFunc_t)BroadcastThread, NULL, &broadcast_thread_attributes);
    error_thread_id = osThreadNew((osThreadFunc_t)ErrorThread, NULL, &error_thread_attributes);
}

/* 
 * Periodic thread function to read the voltage of each cell in the battery
 */
void ReadVoltageThread(void *argument) {
    voltage_thread_counter++;

    // Check for overvoltage and undervoltage conditions in secondary bms
    if (BMSSecondaryFrame0::Instance().GetHighCellVoltage() != 0 &&
        BMSSecondaryFrame0::Instance().GetHighCellVoltage() > bms_config.MAX_CELL_VOLTAGE) {
        Logger::LogError("High cell voltage on cell %d: %dmV", 
                            BMSSecondaryFrame1::Instance().GetHighCellVoltageID(), 
                            BMSSecondaryFrame0::Instance().GetHighCellVoltage());

        // Set high cell voltage error bit
        // This error can only be cleared by a power cycle
    }

    if (BMSSecondaryFrame0::Instance().GetLowCellVoltage() != 0 &&
        BMSSecondaryFrame0::Instance().GetLowCellVoltage() < bms_config.MIN_CELL_VOLTAGE) {
        Logger::LogError("Low cell voltage on cell %d: %dmV", 
                            BMSSecondaryFrame1::Instance().GetLowCellVoltageID(), 
                            BMSSecondaryFrame0::Instance().GetLowCellVoltage());

        // Set low cell voltage error bit
        // This error can only be cleared by a power cycle
    }

    // Reset and update high and low cell voltages from secondary bms
    high_cell_voltage = BMSSecondaryFrame0::Instance().GetHighCellVoltage();
    high_cell_voltage_id = BMSSecondaryFrame1::Instance().GetHighCellVoltageID();
    low_cell_voltage = BMSSecondaryFrame0::Instance().GetLowCellVoltage();
    low_cell_voltage_id = BMSSecondaryFrame1::Instance().GetLowCellVoltageID();

    // Voltage sum for computing average
    // Value is multiplied by 10 because subpack voltage is in 0.01V units, while everything else is in mV
    uint32_t sum_cell_voltage = BMSSecondaryFrame0::Instance().GetSubpackVoltage() * 10;

    // Update cell voltage values in bms
    bms.ReadVoltages();

    for (int i = 0; i < bms_config.NUM_CELLS_PRIMARY; i++) {
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
    avg_cell_voltage = sum_cell_voltage / (bms_config.NUM_CELLS_PRIMARY + bms_config.NUM_CELLS_SECONDARY);

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
 * Periodic thread function to read the current of each cell in the battery
 */
void ReadCurrentThread(void *argument) {
    current_thread_counter++;

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

    // TODO: Figure out how to manage amplifier state
    // HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(CURRENT_EN_GPIO_Port, CURRENT_EN_Pin, GPIO_PIN_RESET);

    // Convert ADC values to current
    float current_l = ADCToCurrentL(adc_vals[0]);
    float current_h = ADCToCurrentH(adc_vals[1]);

    // If current is <50A, use current_l, else use current_h to update global current
    pack_current = current_l < 50.0 ? current_l : current_h;

    // Check if discharge current exceeded
    if (pack_current > bms_config.MAX_DISCHARGE_CURRENT) {
        // Set current error bit
        // This error can only be cleared by a power cycle
        // TODO:

        etl::string<5> float_buf;
        etl::to_string(pack_current, float_buf, format_float, false);
        osMutexAcquire(logger_mutex_id, osWaitForever);
        Logger::LogError("Discharge current limit exceeded: %sA", float_buf.c_str());
        osMutexRelease(logger_mutex_id);
    }

    // Check if charge current exceeded
    if (current_l < -bms_config.MAX_CHARGE_CURRENT) {
        // Set current error bit
        // This error can only be cleared by a power cycle
        // TODO:

        etl::string<5> float_buf;
        etl::to_string(pack_current, float_buf, format_float, false);
        osMutexAcquire(logger_mutex_id, osWaitForever);
        Logger::LogError("Charge current limit exceeded: %sA", float_buf.c_str());
        osMutexRelease(logger_mutex_id);
    }

    // Update running total current integral
    // Convert current to μA, integrate for 0.08s
    if (osMutexAcquire(current_integral_mutex_id, 4) == osOK) {
        integral_current += static_cast<uint16_t>(pack_current * 0.45);
        osMutexRelease(current_integral_mutex_id);
    }

    // If configured, log current every 2.5 seconds
    if (bms_config.LOG_CURRENT) {
        static int counter = 0;
        static constexpr int log_interval = 2500 / read_current_period;
        if (counter++ % log_interval == 0) {
            etl::string<5> float_buf;
            osMutexAcquire(logger_mutex_id, osWaitForever);
            etl::to_string(current_l, float_buf, format_float, false);
            Logger::LogInfo("Current L: %s A", float_buf.c_str());
            etl::to_string(current_h, float_buf, format_float, false);
            Logger::LogInfo("Current H: %s A", float_buf.c_str());
            osMutexRelease(logger_mutex_id);
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
            }

            if (thermistor_vals[i] < thermistor_vals[min_index]) {
                min_index = i;
            }
        }

        // Check for overtemperature conditions
        // TODO: Implement this

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
    // Request to broadcast BMS frame
    osEventFlagsSet(broadcast_event, 0x1);
}

void BroadcastThread(void* argument) {
    static unsigned int broadcast_thread_counter = 0;

    while (1) {
        osEventFlagsWait(broadcast_event, 0x1, osFlagsWaitAny, osWaitForever);

        broadcast_thread_counter++;

        // Capture voltage data
        BMSFrame0::Instance().SetPackVoltage(total_pack_voltage);
        BMSFrame0::Instance().SetAvgCellVoltage(avg_cell_voltage);
        BMSFrame0::Instance().SetHighCellVoltage(high_cell_voltage);
        BMSFrame0::Instance().SetLowCellVoltage(low_cell_voltage);
        BMSFrame1::Instance().SetHighCellVoltageID(high_cell_voltage_id);
        BMSFrame1::Instance().SetLowCellVoltageID(low_cell_voltage_id);

        // Capture current data
        BMSFrame1::Instance().SetPackCurrent(pack_current);

        // Capture then reset current integral
        osMutexAcquire(current_integral_mutex_id, osWaitForever);
        BMSFrame1::Instance().SetIntegralCurrent(integral_current);
        integral_current = 0;
        osMutexRelease(current_integral_mutex_id);

        // Calculate power
        uint16_t avg_power = BMSFrame1::Instance().GetPackCurrent() * BMSFrame0::Instance().GetPackVoltage() / 10;
        BMSFrame1::Instance().SetAveragePower(avg_power);

        // Capture temperature data
        BMSFrame2::Instance().SetHighTemp(high_temp);
        BMSFrame2::Instance().SetLowTemp(low_temp);
        BMSFrame2::Instance().SetHighTempCellID(high_temp_id);
        BMSFrame2::Instance().SetLowTempCellID(low_temp_id);
        BMSFrame2::Instance().SetInternalTemp(internal_temp);

        // Send BMSFrame0, BMSFrame1, BMSFrame2
        CANController::Send(&BMSFrame0::Instance());
        CANController::Send(&BMSFrame1::Instance());
        CANController::Send(&BMSFrame2::Instance());

        // Send BMSFrame3 at 1Hz (every 10th broadcast)
        if (broadcast_thread_counter % 10 == 0) {
            BMSFrame3::Instance().SetFaultFlags(fault_flags);
            BMSFrame3::Instance().SetStatusFlags(status_flags);
            BMSFrame3::Instance().SetPackSoC(0);  // TODO: Implement SoC
            CANController::Send(&BMSFrame3::Instance());
        }
    }
}

void ErrorThread(void* argument) {
    while (1) {
        osEventFlagsWait(error_event, 0x1, osFlagsWaitAny, osWaitForever);

        // Any set bit in fault_flags indicates an error
        if (fault_flags) {
            // Open main contactors
            SetContactorState(3, false);
            SetContactorState(4, false);

            // Update BMSFrame3 with errors
            BMSFrame3::Instance().SetFaultFlags(fault_flags);
            BMSFrame3::Instance().SetStatusFlags(status_flags);

            // Send BMSFrame3
            CANController::Send(&BMSFrame3::Instance());
        }

        // If no errors, close contactors
        else if (fault_flags == 0) {
            SetContactorState(3, true);
            osDelay(500);
            SetContactorState(4, true);
        }
    }
}

void SecondaryFrame0Callback(uint8_t *data) {
    // Update total pack voltage
    total_pack_voltage = local_pack_voltage + BMSSecondaryFrame0::Instance().GetSubpackVoltage();

    // Update average cell voltage
    // TODO: Fix this
    uint32_t sum_cell_voltages = avg_cell_voltage * num_total_cells;

    // Update high and low cell voltages
    // Error checking is done in ReadVoltageThread
    high_cell_voltage = BMSSecondaryFrame0::Instance().GetHighCellVoltage() > high_cell_voltage ? 
                            BMSSecondaryFrame0::Instance().GetHighCellVoltage() : high_cell_voltage;
    low_cell_voltage = BMSSecondaryFrame0::Instance().GetLowCellVoltage() < low_cell_voltage ? 
                            BMSSecondaryFrame0::Instance().GetLowCellVoltage() : low_cell_voltage;
}

void VCUFrameCallback(uint8_t *data) {
    // Update kill flag status
    if (VCUFrame0::Instance().GetKillStatus() == 0) {
        fault_flags &= ~(1 << 7);
    } else {
        fault_flags |= (1 << 7);
    }

    // Trigger error thread
    osEventFlagsSet(error_event, 0x1);
}