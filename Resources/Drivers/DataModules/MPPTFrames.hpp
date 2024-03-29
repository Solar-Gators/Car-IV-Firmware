/*
 * MitsubaFrames.hpp
 *
 *  Created on: March 29, 2024
 *      Author: Matthew Shen
 */

#pragma once

#include "DataModule.hpp"

static constexpr uint16_t MPPT1_BASE_ADDRESS = 0x600;
static constexpr uint16_t MPPT1_BASE_ADDRESS = 0x610;
static constexpr uint16_t MPPT1_BASE_ADDRESS = 0x620;


/* MPPT 1 */
DATAMODULE(

    /* CAN definitions */
    MPPTInputMeasurementsFrame1,
    MPPT1_BASE_ADDRESS + 0,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Input volage (V)
    inline static float GetInputVoltage() {
        float input_voltage;
        memcpy(&input_voltage, &Data()[0], 4);
        return input_voltage
    }

    // Input current (A)
    inline static float GetInputCurrent() {
        float input_current;
        memcpy(&input_current, &Data()[4], 4);
        return input_current;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTOutputMeasurementsFrame1,
    MPPT1_BASE_ADDRESS + 1,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Output volage (V)
    inline static float GetOutputVoltage() {
        float output_voltage;
        memcpy(output_voltage, &Data()[0], 4);
        return output_voltage
    }

    // Output current (A)
    inline static float GetOutputCurrent() {
        float output_current;
        memcpy(output_current, &Data()[4], 4);
        return output_current;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTTemperatureFrame1,
    MPPT1_BASE_ADDRESS + 2,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // MOSFET temperature (C)
    inline static float GetFETTemp() {
        float fet_temp;
        memcpy(&fet_temp, &Data()[0], 4);
        return fet_temp
    }

    // Controller temperature (C)
    inline static float GetControllerTemp() {
        float controller_temp;
        memcpy(&controller_temp, &Data()[4], 4);
        return controller_temp;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTAuxPowerFrame1,
    MPPT1_BASE_ADDRESS + 3,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // 12V supply (V)
    inline static float GetFETTemp() {
        float supply_12V;
        memcpy(&supply_12V, &Data()[0], 4);
        return supply_12V
    }

    // 3V supply (V)
    inline static float GetControllerTemp() {
        float supply_3V;
        memcpy(&supply_3V, &Data()[4], 4);
        return supply_3V;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTLimitsFrame1,
    MPPT1_BASE_ADDRESS + 4,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Max. Output Voltage (V)
    inline static float GetMaxOutputVoltage() {
        float voltage;
        memcpy(&voltage, &Data()[0], 4);
        return voltage
    }

    // Max Input Current (A)
    inline static float GetMaxInputCurrent() {
        float current;
        memcpy(&current, &Data()[4], 4);
        return current;
    }
)


/* MPPT 2 */
DATAMODULE(

    /* CAN definitions */
    MPPTInputMeasurementsFrame2,
    MPPT2_BASE_ADDRESS + 0,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Input volage (V)
    inline static float GetInputVoltage() {
        float input_voltage;
        memcpy(&input_voltage, &Data()[0], 4);
        return input_voltage
    }

    // Input current (A)
    inline static float GetInputCurrent() {
        float input_current;
        memcpy(&input_current, &Data()[4], 4);
        return input_current;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTOutputMeasurementsFrame2,
    MPPT2_BASE_ADDRESS + 1,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Output volage (V)
    inline static float GetOutputVoltage() {
        float output_voltage;
        memcpy(output_voltage, &Data()[0], 4);
        return output_voltage
    }

    // Output current (A)
    inline static float GetOutputCurrent() {
        float output_current;
        memcpy(output_current, &Data()[4], 4);
        return output_current;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTTemperatureFrame2,
    MPPT2_BASE_ADDRESS + 2,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // MOSFET temperature (C)
    inline static float GetFETTemp() {
        float fet_temp;
        memcpy(&fet_temp, &Data()[0], 4);
        return fet_temp
    }

    // Controller temperature (C)
    inline static float GetControllerTemp() {
        float controller_temp;
        memcpy(&controller_temp, &Data()[4], 4);
        return controller_temp;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTAuxPowerFrame2,
    MPPT2_BASE_ADDRESS + 3,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // 12V supply (V)
    inline static float GetFETTemp() {
        float supply_12V;
        memcpy(&supply_12V, &Data()[0], 4);
        return supply_12V
    }

    // 3V supply (V)
    inline static float GetControllerTemp() {
        float supply_3V;
        memcpy(&supply_3V, &Data()[4], 4);
        return supply_3V;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTLimitsFrame2,
    MPPT2_BASE_ADDRESS + 4,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Max. Output Voltage (V)
    inline static float GetMaxOutputVoltage() {
        float voltage;
        memcpy(&voltage, &Data()[0], 4);
        return voltage
    }

    // Max Input Current (A)
    inline static float GetMaxInputCurrent() {
        float current;
        memcpy(&current, &Data()[4], 4);
        return current;
    }
)


/* MPPT 3 */
DATAMODULE(

    /* CAN definitions */
    MPPTInputMeasurementsFrame3,
    MPPT3_BASE_ADDRESS + 0,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Input volage (V)
    inline static float GetInputVoltage() {
        float input_voltage;
        memcpy(&input_voltage, &Data()[0], 4);
        return input_voltage
    }

    // Input current (A)
    inline static float GetInputCurrent() {
        float input_current;
        memcpy(&input_current, &Data()[4], 4);
        return input_current;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTOutputMeasurementsFrame3,
    MPPT3_BASE_ADDRESS + 1,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Output volage (V)
    inline static float GetOutputVoltage() {
        float output_voltage;
        memcpy(output_voltage, &Data()[0], 4);
        return output_voltage
    }

    // Output current (A)
    inline static float GetOutputCurrent() {
        float output_current;
        memcpy(output_current, &Data()[4], 4);
        return output_current;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTTemperatureFrame3,
    MPPT3_BASE_ADDRESS + 2,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // MOSFET temperature (C)
    inline static float GetFETTemp() {
        float fet_temp;
        memcpy(&fet_temp, &Data()[0], 4);
        return fet_temp
    }

    // Controller temperature (C)
    inline static float GetControllerTemp() {
        float controller_temp;
        memcpy(&controller_temp, &Data()[4], 4);
        return controller_temp;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTAuxPowerFrame3,
    MPPT3_BASE_ADDRESS + 3,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // 12V supply (V)
    inline static float GetFETTemp() {
        float supply_12V;
        memcpy(&supply_12V, &Data()[0], 4);
        return supply_12V
    }

    // 3V supply (V)
    inline static float GetControllerTemp() {
        float supply_3V;
        memcpy(&supply_3V, &Data()[4], 4);
        return supply_3V;
    }
)

DATAMODULE(

    /* CAN definitions */
    MPPTLimitsFrame3,
    MPPT3_BASE_ADDRESS + 4,
    CAN_ID_STD,
    CAN_RTR_DATA,
    8,

    // Max. Output Voltage (V)
    inline static float GetMaxOutputVoltage() {
        float voltage;
        memcpy(&voltage, &Data()[0], 4);
        return voltage
    }

    // Max Input Current (A)
    inline static float GetMaxInputCurrent() {
        float current;
        memcpy(&current, &Data()[4], 4);
        return current;
    }
)