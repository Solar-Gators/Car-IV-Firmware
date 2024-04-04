#include "threads.h"

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
    .name = "Read Temperature Thread",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
osTimerId_t temperature_timer_id = osTimerNew((osThreadFunc_t)ReadTemperatureThread, 
                                            osTimerPeriodic, 
                                            NULL, 
                                            &temperature_periodic_timer_attr);

/* Start periodic threads */
void ThreadsStart() {
    // Toggle read voltage thread every 25ms (40Hz)
    osTimerStart(voltage_timer_id, 25);

    // Toggle read current thread every 8ms (125Hz)
    osTimerStart(current_timer_id, 8);

    // Toggle read temperature thread every 100ms (10Hz)
    osTimerStart(temperature_timer_id, 1000);
}

/* 
 * Periodic thread function to read the voltage of each cell in the battery
 */
void ReadVoltageThread(void *argument) {
    bms.ReadVoltages();

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
    }
}

/* 
 * Periodic thread function to read the current of each cell in the battery
 */
void ReadCurrentThread(void *argument) {
}

/* 
 * Periodic thread function to read the temperature of each cell in the battery
 */
void ReadTemperatureThread(void *argument) {
    SetAmplifierState(true);

    HAL_Delay(10);

    for (int i = 0; i < 8; i++) {
        adcs[2].ManualSelectChannel(i);
        adcs[2].StartConversion();

        HAL_Delay(10);

        uint16_t data = 0;
        adcs[2].ReadChannel(i, &data);

        Logger::LogInfo("Thermistor %d: %d", i, data);
    }

    SetAmplifierState(false);
}