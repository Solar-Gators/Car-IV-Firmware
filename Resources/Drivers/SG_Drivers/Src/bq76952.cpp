#include "bq76952.hpp"

void BQ76952::SendData(uint8_t register_address, uint8_t *data) {
    uint16_t devAddress = (I2C_RES_ADDR << 8) | register_address;

    HAL_I2C_Master_Transmit(phi2c_, devAddress, data, 1, HAL_MAX_DELAY);
}