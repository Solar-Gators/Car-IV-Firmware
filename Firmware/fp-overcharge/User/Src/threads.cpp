#include "threads.h"

#include "etl/string.h"
#include "etl/to_string.h"
#include "etl/format_spec.h"

/* Start periodic threads */
void ThreadsStart() {
}

void ProhelionCurrentCallback(uint8_t *data) {
    HAL_GPIO_TogglePin(OK_LED_GPIO_Port, OK_LED_Pin);
    int32_t current = ProhelionCurrentSenseFrame::Instance().GetCurrent();

    // High discharge
    if (current < -50000) {
        HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
        SetContactorState(1, true);
    }
    // High charge
    else if (current > 24000) {
        HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
        SetContactorState(1, true);
    }

    Logger::LogInfo("Current: %d", current);
}