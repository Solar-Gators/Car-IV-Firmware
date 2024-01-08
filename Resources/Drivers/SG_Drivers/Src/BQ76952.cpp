#include "BQ76952.hpp"
#include <cstring>

HAL_StatusTypeDef BQ76952::Init(I2C_HandleTypeDef *hi2c){
    hi2c_ = hi2c;

    return HAL_I2C_IsDeviceReady(hi2c_, BQ_I2C_ADDR_WRITE, 10, 50);
}

HAL_StatusTypeDef BQ76952::ConfigUpdate(bool config_update){
    HAL_StatusTypeDef status;

    if (config_update) {
        status = SubcmdCmdOnly(BQ769X2_SUBCMD_SET_CFGUPDATE);
    }
    else {
        status = SubcmdCmdOnly(BQ769X2_SUBCMD_EXIT_CFGUPDATE);
    }

    if(status != HAL_OK)
    	return status;

    config_update_enabled_ = config_update;

    return HAL_OK;
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
    HAL_StatusTypeDef status;

	int16_t current = 0;
	status = DirectReadI2(BQ769X2_CMD_CURRENT_CC2, &current);
    if(status != HAL_OK)
        return status;

	pack_current_ = current * 0.01; // convert to mA

    return status;
}

HAL_StatusTypeDef BQ76952::ReadTemperatures(){ 
    HAL_StatusTypeDef status;
    int num_temps = 0;
    float sum_temps = 0;
    int16_t temp = 0;

    //read temperatures of thermistors
    for (int i = 0; i < (sizeof(temp_registers)/sizeof(temp_registers[0])); i++){
        status = DirectReadI2(temp_registers[i], &temp); // returns 0.1k
        if (status != HAL_OK) { return status; }
        
        temperatures_[i] = (temp * 10.0) - 273.15; // convert to cesius

        if(i == 0){
            low_temperature_ = temperatures_[i];
            high_temperature_ = temperatures_[i];
        }else{
            if(temperatures_[i] < low_temperature_){
                low_temperature_ = temperatures_[i];
            }
            if(temperatures_[i] > high_temperature_){
                high_temperature_ = temperatures_[i];
            }
        }

        num_temps++;
        sum_temps += temperatures_[i];
    }

    avg_temperature_ = sum_temps / num_temps;

    // read chips internal temperature value
    status = DirectReadI2(BQ769X2_CMD_TEMP_INT, &temp);
    if (status != HAL_OK) { return status; }
    chip_temperature_ = (temp * 10.0) - 273.15;
}

HAL_StatusTypeDef BQ76952::ReadSafetyFaults() {
    uint8_t stat_a_byte;
    uint8_t stat_b_byte;
    uint8_t stat_c_byte;

    HAL_StatusTypeDef status;

    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_A, &stat_a_byte, 1);
    if (status != HAL_OK) 
        return status;
    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_B, &stat_b_byte, 1);
    if (status != HAL_OK) 
        return status;
    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_C, &stat_c_byte, 1);
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

bool BQ76952::GetConfigUpdateStatus(){
	return config_update_enabled_;
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

HAL_StatusTypeDef BQ76952::SubcmdReadF4(const uint16_t subcmd, float *value){
	float f32;
	HAL_StatusTypeDef status = SubcmdRead(subcmd, (uint32_t *)&f32, 4);

    if (status != HAL_OK)
        return status;

    *value = f32;
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
    return SubcmdWrite(subcmd, 0, 0);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteU1(const uint16_t subcmd, uint8_t value) {
    return SubcmdWrite(subcmd, value, 1);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteU2(const uint16_t subcmd, uint16_t value) {
    return SubcmdWrite(subcmd, value, 2);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteU4(const uint16_t subcmd, uint32_t value) {
    return SubcmdWrite(subcmd, value, 4);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteI1(const uint16_t subcmd, int8_t value) {
    return SubcmdWrite(subcmd, value, 1);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteI2(const uint16_t subcmd, int16_t value) {
    return SubcmdWrite(subcmd, value, 2);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteI4(const uint16_t subcmd, int32_t value) {
    return SubcmdWrite(subcmd, value, 4);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteF4(const uint16_t subcmd, float value) {
    uint32_t *u32 = (uint32_t *)&value;
    return SubcmdWrite(subcmd, *u32, 4);
}

HAL_StatusTypeDef BQ76952::DatamemReadU1(const uint16_t reg_addr, uint8_t *value){
	if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr)){
		return HAL_ERROR;
	}
	uint32_t u32;
	HAL_StatusTypeDef status = SubcmdRead(reg_addr, &u32, 1);
	if(status == HAL_OK){
		*value = (uint8_t)u32;
	}
	return status;
}
HAL_StatusTypeDef BQ76952::DatamemWriteU1(const uint16_t reg_addr, uint8_t value){
	if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !config_update_enabled_){
		return HAL_ERROR;
	}
	return SubcmdWrite(reg_addr, value, 1);
}
HAL_StatusTypeDef BQ76952::DatamemWriteU2(const uint16_t reg_addr, uint16_t value){
	if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !config_update_enabled_){
		return HAL_ERROR;
	}
	return SubcmdWrite(reg_addr, value, 2);
}
HAL_StatusTypeDef BQ76952::DatamemWriteI1(const uint16_t reg_addr, int8_t value){
	if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !config_update_enabled_){
		return HAL_ERROR;
	}
	return SubcmdWrite(reg_addr, value, 1);
}
HAL_StatusTypeDef BQ76952::DatamemWriteI2(const uint16_t reg_addr, int16_t value){
	if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !config_update_enabled_){
		return HAL_ERROR;
	}
	return SubcmdWrite(reg_addr, value, 2);
}
HAL_StatusTypeDef BQ76952::DatamemWriteF4(const uint16_t reg_addr, float value){
	if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !config_update_enabled_){
		return HAL_ERROR;
	}

	uint32_t *u32 = (uint32_t *)&value;

	return SubcmdWrite(reg_addr, *u32, 4);
}

HAL_StatusTypeDef BQ76952::EnableThermistorPins(){
	HAL_StatusTypeDef status;

	// need to talk to Matthew about what settins to configure thermistor to (18 or 180 kOhm, which temperature model to use)
    // current value (0b00000111) sets for 18k, 18k curve

	//set ALERT for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_ALERT, 0b00000111);
    if(status != HAL_OK)
    	return status;

	//set TS1 for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_TS1, 0b00000111);
    if(status != HAL_OK)
    	return status;

	//set TS2 for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_TS2, 0b00000111);
    if(status != HAL_OK)
    	return status;

	//set TS3 for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_TS3, 0b00000111);
    if(status != HAL_OK)
    	return status;

	//set HDQ for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_HDQ, 0b00000111);
    if(status != HAL_OK)
    	return status;

    //set CFETOFF for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_CFETOFF, 0b00000111);
    if(status != HAL_OK)
    	return status;

    //set DFETOFF for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_DFETOFF, 0b00000111);
    if(status != HAL_OK)
    	return status;

    //set DCHG for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_DCHG, 0b00000111);
    if(status != HAL_OK)
    	return status;

    //set DDSG for 18k, 18k curve, for use in measuring cell
	status = DatamemWriteU1(BQ769X2_SET_CONF_DDSG, 0b00000111);
    if(status != HAL_OK)
    	return status;

    return HAL_OK;
}
