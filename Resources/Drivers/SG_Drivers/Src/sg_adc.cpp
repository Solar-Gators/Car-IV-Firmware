/*
 * ADS7138 ADC I2C Driver
 *
 * Created on: 1/12/2024
 * Authors: Anthony Kfoury, Braden Azis
 */
#include "sg_adc.hpp"

/**Configure and reset the device*/
uint8_t ADS7138_Init(ADCDevice *adc, I2C_HandleTypeDef *i2cHandle){
    adc -> i2c= i2cHandle;
    adc -> maxchannelData[0]=0;
    adc -> maxchannelData[1]=0;
    adc -> maxchannelData[2]=0;
    adc -> maxchannelData[3]=0;
    adc -> maxchannelData[4]=0;
    adc -> maxchannelData[5]=0;
    adc -> maxchannelData[6]=0;
    adc -> maxchannelData[7]=0;

    adc -> maxfchannelData[0]=0.0f;
    adc -> maxfchannelData[1]=0.0f;
    adc -> maxfchannelData[2]=0.0f;
    adc -> maxfchannelData[3]=0.0f;
    adc -> maxfchannelData[4]=0.0f;
    adc -> maxfchannelData[5]=0.0f;
    adc -> maxfchannelData[6]=0.0f;
    adc -> maxfchannelData[7]=0.0f;

    adc -> minchannelData[0]=0;
    adc -> minchannelData[1]=0;
    adc -> minchannelData[2]=0;
    adc -> minchannelData[3]=0;
    adc -> minchannelData[4]=0;
    adc -> minchannelData[5]=0;
    adc -> minchannelData[6]=0;
    adc -> minchannelData[7]=0;

    adc -> minfchannelData[0]=0.0f;
    adc -> minfchannelData[1]=0.0f;
    adc -> minfchannelData[2]=0.0f;
    adc -> minfchannelData[3]=0.0f;
    adc -> minfchannelData[4]=0.0f;
    adc -> minfchannelData[5]=0.0f;
    adc -> minfchannelData[6]=0.0f;
    adc -> minfchannelData[7]=0.0f;

    //error detection
    HAL_StatusTypeDef status;
    uint8_t errors=0;

    // configure device
    uint8_t data =0b00000100;
    status = ADC_WriteRegister(adc, GENERAL_CFG, &data);
    errors += (status!=HAL_OK);


    data =0b00000000;
    status = ADC_WriteRegister(adc, PIN_CFG, &data);
    errors += (status != HAL_OK);

    data =0b00010000;
    status = ADC_WriteRegister(adc, OPMODE_CFG, &data);
    errors += (status != HAL_OK);

    data =0b00010001;
    status = ADC_WriteRegister(adc, SEQUENCE_CFG, &data);
    errors += (status != HAL_OK);

    data =0b00000000;
    status = ADC_WriteRegister(adc, ALERT_PIN_CFG, &data);
    errors += (status != HAL_OK);

    return errors;
}

/**Updates all the minChannelData in the ADCDevice*/
HAL_StatusTypeDef ADC_UpdateAllMinChannel(ADCDevice *adc){
    HAL_StatusTypeDef status;
    for(int i = 0; i < 8; i++){
        if((status = ADC_UpdateMinChannel(adc, i)) != HAL_OK) return status;     
    }
}

/**Updates all the maxchannelData in the ADCDevice*/
HAL_StatusTypeDef ADC_UpdateAllMaxChannel(ADCDevice *adc){
    HAL_StatusTypeDef status;
    for(int i = 0; i < 8; i++){
        if((status = ADC_UpdateMaxChannel(adc, i)) != HAL_OK) return status;     
    }
    return HAL_OK;
}

/** Update the channel data in the ADCDevice to the minimum and resets the corresponding data registers*/
HAL_StatusTypeDef ADC_UpdateMinChannel(ADCDevice *adc, uint8_t channelNumber){
    if(channelNumber < 0 || channelNumber > 7){
        return HAL_ERROR;
    }
    uint8_t MSB, LSB;
    HAL_StatusTypeDef status;

    if((status = ADC_ReadRegister(adc, MIN_CH0_MSB + 2 * channelNumber, &MSB)) != HAL_OK) return status;
    if((status = ADC_ReadRegister(adc, MIN_CH0_LSB + 2 * channelNumber, &LSB)) != HAL_OK) return status;

    uint16_t ChannelValue= ((uint16_t)MSB) << 8 | LSB; // Get total data from MSB and LSB

    // Update changes in the ADCDevice
    adc->minchannelData[channelNumber] = ChannelValue;
    adc->minfchannelData[channelNumber] = (float)ChannelValue / 0xFFF;
    return HAL_OK;
}

/** Update the channel data in the ADCDevice to the maximum and resets the corresponding data registers*/
HAL_StatusTypeDef ADC_UpdateMaxChannel(ADCDevice *adc, uint8_t channelNumber){
    if(channelNumber < 0 || channelNumber > 7){
        return HAL_ERROR;
    }
    uint8_t MSB, LSB;
    HAL_StatusTypeDef status;

    if((status = ADC_ReadRegister(adc, MAX_CH0_MSB + 2 * channelNumber, &MSB)) != HAL_OK) return status;
    if((status = ADC_ReadRegister(adc, MAX_CH0_LSB + 2 * channelNumber, &LSB)) != HAL_OK) return status;

    uint16_t ChannelValue = ((uint16_t)MSB) << 8 | LSB; // Get total data from MSB and LSB

    // Update changes in the ADCDevice
    adc->maxchannelData[channelNumber] = ChannelValue;
    adc->maxfchannelData[channelNumber] = (float)ChannelValue / 0xFFF;

    return HAL_OK;
}
/**Resets Min Channels to 0xFF and Max Channels to 0x0*/
HAL_StatusTypeDef ADC_ResetChannels(ADCDevice *adc){
    for(int i = 0; i < 8; i++){
        HAL_StatusTypeDef status;
        uint8_t MSB;
        if((status = ADC_ReadRegister(adc, MAX_CH0_MSB + i * 2, &MSB)) != HAL_OK) return status;
        if((status = ADC_ReadRegister(adc, MAX_CH0_LSB + i * 2, &MSB)) != HAL_OK) return status;      
    }
    return HAL_OK;
}

/**Read a register in the ADC Device*/
HAL_StatusTypeDef ADC_ReadRegister(ADCDevice *adc, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Read(adc->i2c, ADS7138_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**Write to a register in the ADC Device*/
HAL_StatusTypeDef ADC_WriteRegister(ADCDevice *adc, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Write(adc->i2c, ADS7138_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}