#include "TMC4671.hpp"

TMC4671::TMC4671(SPI_HandleTypeDef *phspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) :
    hspi_(phspi), cs_port_(cs_port), cs_pin_(cs_pin)
{
    // Set chip select high
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

/*Initilization Function
 -  0x0Ch - DS_ANALOG_INPUT_STAGE_CFG,
    - ADC I0 INP vs GND
    - ADC I1 INP vs GND
    - Rest of valuees leave at all zeros
*/
//montitoring
//control
//testing functions

uint8_t TMC4671::INIT(sdfgfdsdfghgfdsdfghj){
    HAL_StatusTypeDef status;
    //setup data
    status = WriteReg8(DS_ANALOG_INPUT_STAGE_CFG, data);
    if(status != HAL_OK){
        return(1)
    }

    //setup data
    status = WriteReg8(DS_ANALOG_INPUT_STAGE_CFG, data);
    if(status != HAL_OK){
        return(2)
    }



}









HAL_StatusTypeDef TMC4671::ReadReg8(TMC4671_Register reg, uint8_t *data) {

    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Receive(hspi_, data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return(status);
}
HAL_StatusTypeDef TMC4671::ReadReg16(TMC4671_Register reg, uint16_t *data) {

    HAL_StatusTypeDef status;
    uint8_t rx_data[2];
    uint16_t formated;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Receive(hspi_, rx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    formated = ((rx_data[0] << 8) | rx_data[1]);
    *data = formated;

    return(status);
}
HAL_StatusTypeDef TMC4671::ReadReg32(TMC4671_Register reg, uint32_t *data) {

    HAL_StatusTypeDef status;
    uint8_t rx_data[4];
    uint32_t formated;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Receive(hspi_, rx_data, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    formated = ((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);
    *data = formated;
    
    return(status);
}

HAL_StatusTypeDef TMC4671::WriteReg8(TMC4671_Register reg, uint8_t data) {

    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi_, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return(status);

}
HAL_StatusTypeDef TMC4671::WriteReg16(TMC4671_Register reg, uint16_t data) {

    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi_, (uint8_t*)&data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return(status);
}
HAL_StatusTypeDef TMC4671::WriteReg32(TMC4671_Register reg, uint32_t data) {

    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi_, (uint8_t*)&data, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return(status);
}
