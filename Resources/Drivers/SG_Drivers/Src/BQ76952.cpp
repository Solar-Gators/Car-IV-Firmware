#include "BQ76952.hpp"
#include <cstring>

HAL_StatusTypeDef BQ76952::Init(I2C_HandleTypeDef *hi2c){
    hi2c_ = hi2c;

    return HAL_I2C_IsDeviceReady(hi2c_, BQ_I2C_ADDR_WRITE, 10, 50);
}

HAL_StatusTypeDef BQ76952::ReadVoltages() {
    HAL_StatusTypeDef status = HAL_OK;

    int16_t voltage_sum = 0;
    low_cell_voltage_ = 0;
    high_cell_voltage_ = 10;
    for (int i = 0; i < 16; i++) {
        status = DirectReadI2(BQ769X2_CMD_VOLTAGE_CELL_1 + i*2, &cell_voltages_[i]);
        if (status != HAL_OK) { return status; }

        if (cell_voltages_[i] > high_cell_voltage_) {
            high_cell_voltage_ = cell_voltages_[i];
        }
        else if (cell_voltages_[i] < low_cell_voltage_) {
            low_cell_voltage_ = cell_voltages_[i];
        }

        voltage_sum += cell_voltages_[i];
    }

    avg_cell_voltage_ = voltage_sum / 16;

    status = DirectReadI2(BQ769X2_CMD_VOLTAGE_PACK, &pack_voltage_);
    if (status != HAL_OK) { return status; }

    status = DirectReadI2(BQ769X2_CMD_VOLTAGE_STACK, &pack_voltage_);
    if (status != HAL_OK) { return status; }

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::ReadCurrent(){
	int16_t current = 0;
	DirectReadI2(BQ769X2_CMD_CURRENT_CC2, &current);

	pack_current_ = current * 1e-2F;
}

HAL_StatusTypeDef BQ76952::ReadSafetyFaults() {
    uint8_t stat_a_byte;
    uint8_t stat_b_byte;
    uint8_t stat_c_byte;

    HAL_StatusTypeDef status = HAL_OK;

    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_A, &stat_a_byte, 1);
    if (status != HAL_OK) 
        return status;
    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_B, &stat_b_byte, 1);
    if (status != HAL_OK) 
        return status;
    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_C, &stat_c_byte, 1);
    if (status != HAL_OK) 
        return status;

    



    if (status != HAL_OK) 
        return status;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::Shutdown() {
    return SubcmdCmdOnly(BQ769X2_SUBCMD_SHUTDOWN);
}

int16_t BQ76952::GetCellVoltage(uint32_t cell_num) {
    if (cell_num > 16) {
        return -1;
    }

    return cell_voltages_[cell_num];
}

int16_t BQ76952::GetPackVoltage() {
    return pack_voltage_;
}

int16_t BQ76952::GetStackVoltage() {
    return stack_voltage_;
}

int16_t BQ76952::GetAvgCellVoltage() {
    return avg_cell_voltage_;
}

int16_t BQ76952::GetHighCellVoltage() {
    return high_cell_voltage_;
}

int16_t BQ76952::GetLowCellVoltage() {
    return low_cell_voltage_;
}

HAL_StatusTypeDef BQ76952::WriteBytes(const uint8_t reg_addr, const uint8_t *data, const size_t num_bytes) {
    uint8_t buf[5];

    if (num_bytes > 4){
        return HAL_ERROR;
    }

    buf[0] = reg_addr;
    memcpy(buf + 1, data, num_bytes);

    return HAL_I2C_Master_Transmit(hi2c_, BQ_I2C_ADDR_WRITE, buf, num_bytes + 1, 1000);
}

HAL_StatusTypeDef BQ76952::ReadBytes(uint8_t reg_addr, uint8_t *data, const size_t num_bytes) {
    HAL_I2C_Master_Transmit(hi2c_, BQ_I2C_ADDR_READ, &reg_addr, 1, 1000);
    
    return HAL_I2C_Master_Receive(hi2c_, BQ_I2C_ADDR_READ, data, num_bytes, 1000);
}

HAL_StatusTypeDef BQ76952::DirectReadU1(const uint8_t reg_addr, uint8_t *value){
	uint8_t *buf;

	HAL_StatusTypeDef status = ReadBytes(reg_addr, buf, 1);

    if (status != HAL_OK)
        return status;

    *value = *buf;

    return HAL_OK;
}
HAL_StatusTypeDef BQ76952::DirectReadI1(const uint8_t reg_addr, int8_t *value){
    uint8_t *buf;

    HAL_StatusTypeDef status = ReadBytes(reg_addr, buf, 1);

    if (status != HAL_OK)
        return status;

    *value = (int8_t)(*buf);

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::DirectReadU2(const uint8_t reg_addr, uint16_t *value) {
    uint8_t buf[2];

    HAL_StatusTypeDef status = ReadBytes(reg_addr, buf, 2);

    if (status != HAL_OK) 
        return status;

    *value = (buf[1] << 8) | buf[0];

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::DirectReadI2(const uint8_t reg_addr, int16_t *value) {
    uint8_t buf[2];

    HAL_StatusTypeDef status = ReadBytes(reg_addr, buf, 2);

    if (status != HAL_OK) 
        return status;

    *value = (int16_t)((buf[1] << 8) | buf[0]);

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdRead(const uint16_t subcmd, uint32_t *value, const size_t num_bytes) {
    static uint8_t buf_data[0x20];

	uint8_t buf_subcmd[2] = { (uint8_t)subcmd, (uint8_t)(subcmd >> 8)}; // put subcmd into a buffer

	HAL_StatusTypeDef status = WriteBytes(BQ769X2_CMD_SUBCMD_LOWER, buf_subcmd, 2);
    if (status != HAL_OK)
        return status;

	HAL_Delay(1);

	int num_tries = 0;
	while(1){
		status = ReadBytes(BQ769X2_CMD_SUBCMD_LOWER, buf_data, 2);

		if (status != HAL_OK){
		    return status;
		} else if(num_tries > 10){
			return HAL_ERROR;
		}
		else {
			if(buf_subcmd[0] != buf_data[0] || buf_subcmd[1] != buf_data[1]){
				HAL_Delay(1);
				num_tries++;
			}else{
				break;
			}
		}
	}

	uint8_t data_length;

	status = ReadBytes(BQ769X2_SUBCMD_DATA_LENGTH, &data_length, 1);
    if (status != HAL_OK)
        return status;

	data_length -= 4; // subtract subcmd + checksum + length bytes

	if(data_length > 0x20 || num_bytes > 4){
		return HAL_ERROR; // error
	}

	*value = 0;
	status = ReadBytes(BQ769X2_SUBCMD_DATA_START, buf_data, data_length);
    if (status != HAL_OK)
        return status;

	for(uint8_t i = 0; i < num_bytes; i++){
		*value += buf_data[i] << (i * 8);
	}

	return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadU1(const uint16_t subcmd, uint8_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 1);

    if (status != HAL_OK) 
        return status;

    *value = (uint8_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadU2(const uint16_t subcmd, uint16_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 2);

    if (status != HAL_OK) 
        return status;

    *value = (uint16_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadU4(const uint16_t subcmd, uint32_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 4);

    if (status != HAL_OK)
        return status;

    *value = temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadI1(const uint16_t subcmd, int8_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 1);

    if (status != HAL_OK) 
        return status;

    *value = (int8_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadI2(const uint16_t subcmd, int16_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 2);

    if (status != HAL_OK) 
        return status;

    *value = (int16_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadI4(const uint16_t subcmd, int32_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 4);

    if (status != HAL_OK) 
        return status;

    *value = (int32_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdWrite(const uint16_t subcmd, const uint32_t value, const size_t num_bytes) {
    uint8_t buf_data[4];
    uint8_t buf_subcmd[2] = { subcmd & 0x00FF, subcmd >> 8 };

    HAL_StatusTypeDef status = WriteBytes(BQ769X2_CMD_SUBCMD_LOWER, buf_subcmd, 2);
    if(status != HAL_OK)
    	return status;

    if (num_bytes > 4){
        return HAL_ERROR;
    }
    if (num_bytes > 0){
    	for(int i = 0; i < num_bytes; i++){
    		buf_data[i] = (value >> (i * 8) & 0x000000FF);
    	}
    	status = WriteBytes(BQ769X2_SUBCMD_DATA_START, buf_data, num_bytes);
    }

    return status;
}

HAL_StatusTypeDef BQ76952::SubcmdCmdOnly(const uint16_t subcmd) {
    return SubcmdWrite(BQ769X2_CMD_SUBCMD_LOWER, buf_subcmd, 2);
}

HAL_StatusTypeDef EnableThermistorPins(){

}
