#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>

#include "main.h"

#include "cmsis_os.h"
#include "logger.hpp"
#include "sg_can.hpp"
#include "BQ76952.hpp"
#include "ADS7138.hpp"

/* Datamodules */
#include "CustomBMSFrames.hpp"
#include "VCUFrame.hpp"
#include "DriverControls.hpp"

/* User configuration values */
// TODO: Store configuration in EEPROM
struct BMSConfig {
    int CELL_OFFSET;
    int NUM_CELLS_PRIMARY;
    int NUM_CELLS_SECONDARY;
    int NUM_THERMISTORS_PRIMARY;
    int NUM_THERMISTORS_SECONDARY;
    int16_t MAX_CELL_VOLTAGE;
    int16_t MIN_CELL_VOLTAGE;
    float MAX_DISCHARGE_CURRENT;
    float MAX_CHARGE_CURRENT;
    float MAX_CHARGE_TEMP;
    float MAX_DISCHARGE_TEMP;
    bool LOG_VOLTAGE;
    bool LOG_CURRENT;
    bool LOG_TEMPERATURE;
};
const struct BMSConfig bms_config = {
    .CELL_OFFSET = 0,
    .NUM_CELLS_PRIMARY = 16,
    .NUM_CELLS_SECONDARY = 0,
    .NUM_THERMISTORS_PRIMARY = 20,
    .NUM_THERMISTORS_SECONDARY = 0,
    .MAX_CELL_VOLTAGE = 4200,
    .MIN_CELL_VOLTAGE = 2500,
    .MAX_DISCHARGE_CURRENT = 100.0,
    .MAX_CHARGE_CURRENT = 21.775,
    .MAX_CHARGE_TEMP = 60.0,
    .MAX_DISCHARGE_TEMP = 45.0,
    .LOG_VOLTAGE = true,
    .LOG_CURRENT = true,
    .LOG_TEMPERATURE = true,
};

/* Device handles */
extern BQ76952 bms;
extern ADS7138 adcs[3];

enum class ContactorSource_Type : int {
    SUPPLEMENTAL = 0,
    MAIN = 1,
};

void SetAmplifierState(bool state);
void SetContactorSource(ContactorSource_Type source);
void SetContactorState(uint8_t contactor, bool state);
float ADCToTemp(uint16_t adc_val);
float ADCToCurrentL(uint16_t adc_val);
float ADCToCurrentH(uint16_t adc_val);

/**
 * ADC Mappings
 * adc[0]
 *  - CH0 - TMP4
 *  - CH1 - TMP1
 *  - CH2 - TMP3
 *  - CH3 - TMP2
 *  - CH4 - TMP6
 *  - CH5 - CURRENT_L
 *  - CH6 - TMP5
 *  - CH7 - CURRENT_H
 * adc[1]
 *  - CH0 - TMP11
 *  - CH1 - TMP12
 *  - CH2 - TMP9
 *  - CH3 - TMP10
 *  - CH4 - TMP14
 *  - CH5 - TMP7
 *  - CH6 - TMP13
 *  - CH7 - TMP8
 * adc[2]
 * - CH0 - TMP20
 * - CH0 - TMP19
 * - CH0 - TMP21 (4.7k dummy)
 * - CH0 - TMP22 (on-board thermistor)
 * - CH0 - TMP16
 * - CH0 - TMP15
 * - CH0 - TMP18
 * - CH0 - TMP17
 * 
*/
constexpr uint8_t MapADCChannelToThermistor(uint8_t adc, uint8_t channel) {
    // Return 255 when channel does not map to a thermistor
    switch (adc) {
        case 0:
            switch (channel) {
            case 0: return 4;
            case 1: return 1;
            case 2: return 3;
            case 3: return 2;
            case 4: return 6;
            case 5: return 255;
            case 6: return 5;
            case 7: return 255;
            default: return 255;
            }
        case 1:
            switch (channel) {
            case 0: return 11;
            case 1: return 12;
            case 2: return 9;
            case 3: return 10;
            case 4: return 14;
            case 5: return 7;
            case 6: return 13;
            case 7: return 8;
            default: return 255;
            }
        case 2:
            switch (channel) {
            case 0: return 20;
            case 1: return 19;
            case 2: return 21;
            case 3: return 22;
            case 4: return 16;
            case 5: return 15;
            case 6: return 18;
            case 7: return 17;
            default: return 255;
            }
        default: return 255;
    }
}

#endif /* USER_HPP_ */