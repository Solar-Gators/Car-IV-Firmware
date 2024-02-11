#include "DACx311.hpp"

DACx311::DACx311(SPI_HandleTypeDef *phspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) :
    hspi_(phspi), cs_port_(cs_port), cs_pin_(cs_pin)
{}

HAL_StatusTypeDef DACx311::SetValue(uint16_t value) {
    return Write(DACx311_MODE_NORMAL, value);
}

HAL_StatusTypeDef DACx311::Sleep() {
    return Write(DACx311_MODE_POWERDOWN_HIZ, 0);
}

HAL_StatusTypeDef DACx311::Write(DACx311_ModeTypeDef mode, uint16_t data) {
    HAL_StatusTypeDef status;
    uint16_t tx_data;

    // Refer to datasheet for formatting information
    tx_data = ((uint16_t)mode << 14) | ((data & 0xFFF0) >> 2);

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi_, (uint8_t*)&tx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return status;
}