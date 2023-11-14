#include "LTC1661.hpp"


LTC1661::LTC1661(SPI_HandleTypeDef *phspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    phspi_ = phspi;
    cs_port_ = cs_port;
    cs_pin_ = cs_pin;
}

LTC1661::LTC1661(SPI_HandleTypeDef *phspi) {
    phspi_ = phspi;
}

HAL_StatusTypeDef LTC1661::Wake() {
    return Write(0x0D, 0x00);
}

HAL_StatusTypeDef LTC1661::Sleep() {
    return Write(0x0E, 0x00);
}

HAL_StatusTypeDef LTC1661::LoadA(uint16_t value) {
    return Write(0x01, value);
}

HAL_StatusTypeDef LTC1661::LoadB(uint16_t value) {
    return Write(0x02, value);
}

HAL_StatusTypeDef LTC1661::LoadAB(uint16_t value) {
    return Write(0x0F, value);
}

HAL_StatusTypeDef LTC1661::Write(uint8_t cmd, uint16_t data) {
    HAL_StatusTypeDef status = HAL_OK;

    if (cs_port_)
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

    uint8_t bytes[2] = {(uint8_t)((cmd << 4) | ((data >> 6) & 0x0F)), (uint8_t)((data << 2) & 0xFF)};

    status = HAL_SPI_Transmit(phspi_, bytes, 2, HAL_MAX_DELAY);

    if (cs_port_)
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return status;
}