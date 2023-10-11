/*
 * MPPT.cpp
 *
 *  Created on: Apr 17, 2023
 *      Author: Jack W
 */
#include "Mppt.hpp"
#include "DataModuleInfo.hpp"

namespace SolarGators::DataModules
{

union float2byte
{
	  float f;
	  char  s[4];
};

float2byte f2b;

enum {
	MPPT0 = 1,
	MPPT1 = 2,
	MPPT2 = 3
} mpptNumber;

Mpptx0::Mpptx0(uint32_t can_id):
		DataModule(can_id, 0, 8),
		inputVoltage(0),
		inputCurrent(0) // unsure if i need to do this, orionBMS doesnt but steering does
		{}

void Mpptx0::ToByteArray(uint8_t* buff) const
{
	f2b.f = inputVoltage;
	for (int i=0;i<=3;i++){
		buff[i] = f2b.s[i];
	}
	f2b.f = inputCurrent;
	for (int i=4;i<=7;i++){
		buff[i] = f2b.s[i];
	}
}

void Mpptx0::FromByteArray(uint8_t* buff)
{
	for(int i=0;i<=3;i++){
		f2b.s[i] = buff[i];
	}
	inputVoltage = f2b.f;
	for(int i=4;i<=7;i++){
		f2b.s[i-4] = buff[i];
	}
	inputCurrent = f2b.f;
}

float Mpptx0::getInputVoltage() const {
	return inputVoltage;
}

float Mpptx0::getInputCurrent() const {
	return inputCurrent;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx0::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX0_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX0_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX0_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx0::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("inputVoltage", getInputVoltage());
		http.addData("inputCurrent", getInputCurrent());
		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx0", http.getParameters());
		http.flush();
	}
#endif

Mpptx1::Mpptx1(uint32_t can_id): // INCREMENT BY 1 FROM MPPTx0
		DataModule(can_id, 0, 8),
		outputVoltage(0),
		outputCurrent(0) // unsure if i need to do this, orionBMS doesnt but steering does
{}

void Mpptx1::ToByteArray(uint8_t* buff) const
{
f2b.f = outputVoltage;
for (int i=0;i<=3;i++){
	buff[i] = f2b.s[i];
}
f2b.f = outputCurrent;
for (int i=4;i<=7;i++){
	buff[i] = f2b.s[i-4];
}
}

void Mpptx1::FromByteArray(uint8_t* buff)
{
	for(int i=0;i<=3;i++){
		f2b.s[i] = buff[i];
	}
	outputVoltage = f2b.f;
	for(int i=4;i<=7;i++){
		f2b.s[i-4] = buff[i];
	}
	outputCurrent = f2b.f;
}

float Mpptx1::getOutputVoltage() const {
	return outputVoltage;
}

float Mpptx1::getOutputCurrent() const {
	return outputCurrent;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx1::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX1_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX1_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX1_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx1::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("outputVoltage", getOutputVoltage());
		http.addData("outputCurrent", getOutputCurrent());
		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx1", http.getParameters());
		http.flush();
	}
#endif

Mpptx2::Mpptx2(uint32_t can_id): // INCREMENT BY 2 FROM MPPTx0
	DataModule(can_id, 0, 8),
	mosfetTemp(0),
	controllerTemp(0) // unsure if i need to do this, orionBMS doesnt but steering does
{}

void Mpptx2::ToByteArray(uint8_t* buff) const
{
	f2b.f = mosfetTemp;
	for (int i=0;i<=3;i++){
		buff[i] = f2b.s[i];
	}
	f2b.f = controllerTemp;
	for (int i=4;i<=7;i++){
		buff[i] = f2b.s[i];
	}
}

void Mpptx2::FromByteArray(uint8_t* buff)
{
	for(int i=0;i<=3;i++){
		f2b.s[i] = buff[i];
	}
	mosfetTemp = f2b.f;
	for(int i=4;i<=7;i++){
		f2b.s[i] = buff[i];
	}
	controllerTemp = f2b.f;
}

float Mpptx2::getMosfetTemp() const {
	return mosfetTemp;
}

float Mpptx2::getControllerTemp() const {
	return controllerTemp;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx2::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX2_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX2_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX2_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx2::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("mosfetTemp", getMosfetTemp());
		http.addData("controllerTemp", getControllerTemp());
		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx2", http.getParameters());
		http.flush();
	}
#endif

Mpptx3::Mpptx3(uint32_t can_id): // INCREMENT BY 3 FROM MPPTx0
	DataModule(can_id, 0, 8),
	aux12V(0),
	aux3V(0) // unsure if i need to do this, orionBMS doesnt but steering does
{}

void Mpptx3::ToByteArray(uint8_t* buff) const
{
	f2b.f = aux12V;
	for (int i=0;i<=3;i++){
		buff[i] = f2b.s[i];
	}
	f2b.f = aux3V;
	for (int i=4;i<=7;i++){
		buff[i] = f2b.s[i];
	}
}

void Mpptx3::FromByteArray(uint8_t* buff)
{
	for(int i=0;i<=3;i++){
		f2b.s[i] = buff[i];
	}
	aux12V = f2b.f;
	for(int i=4;i<=7;i++){
		f2b.s[i] = buff[i];
	}
	aux3V = f2b.f;
}

float Mpptx3::getAux12V() const {
	return aux12V;
}

float Mpptx3::getAux3V() const {
	return aux3V;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx3::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX3_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX3_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX3_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx3::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("aux12v", getAux12V());
		http.addData("aux3v", getAux3V());
		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx3", http.getParameters());
		http.flush();
	}
#endif

Mpptx4::Mpptx4(uint32_t can_id): // INCREMENT BY 4 FROM MPPTx0
	DataModule(can_id, 0, 8),
	maxOutputVoltage(0),
	maxInputCurrent(0) // unsure if i need to do this, orionBMS doesnt but steering does
{}

void Mpptx4::ToByteArray(uint8_t* buff) const
{
	f2b.f = maxOutputVoltage;
	for (int i=0;i<=3;i++){
		buff[i] = f2b.s[i];
	}
	f2b.f = maxInputCurrent;
	for (int i=4;i<=7;i++){
		buff[i] = f2b.s[i];
	}
}

void Mpptx4::FromByteArray(uint8_t* buff)
{
	for(int i=0;i<=3;i++){
		f2b.s[i] = buff[i];
	}
	maxOutputVoltage = f2b.f;
	for(int i=4;i<=7;i++){
		f2b.s[i] = buff[i];
	}
	maxInputCurrent = f2b.f;
}

float Mpptx4::getMaxOutputVoltage() const {
	return maxOutputVoltage;
}

float Mpptx4::getMaxInputCurrent() const {
	return maxInputCurrent;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx4::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX4_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX4_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX4_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx4::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("maxOutputVoltage", getMaxOutputVoltage());
		http.addData("maxInputCurrent", getMaxInputCurrent());
		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx4", http.getParameters());
		http.flush();
	}
#endif


Mpptx5::Mpptx5(uint32_t can_id): // INCREMENT BY 5 FROM MPPTx0
	DataModule(can_id, 0, 8),
	CANRXerr(0),
	CANTXerr(0),
	CANTXoverflow(0),
	mode(0),
	reserved(0),
	counter(0)
{}

void Mpptx5::ToByteArray(uint8_t* buff) const
{
	buff[0]= CANRXerr;
	buff[1]= CANTXerr;
	buff[2]= CANTXoverflow;

	buff[3] = (static_cast<uint8_t>(error_low_array_power)		<< 0);
	buff[3] |= (static_cast<uint8_t>(error_mosfet_overheat)		<< 1);
	buff[3] |= (static_cast<uint8_t>(error_battery_low)			<< 2);
	buff[3] |= (static_cast<uint8_t>(error_battery_full)		<< 3);
	buff[3] |= (static_cast<uint8_t>(error_12v_undervolt)		<< 4);
	buff[3] |= (static_cast<uint8_t>(error_hw_overcurrent)		<< 6);
	buff[3] |= (static_cast<uint8_t>(error_hw_overvolt)			<< 7);

	buff[4] = (static_cast<uint8_t>(flag_input_current_min)		<< 0);
	buff[4] |= (static_cast<uint8_t>(flag_input_current_max)	<< 1);
	buff[4] |= (static_cast<uint8_t>(flag_output_voltage_max)	<< 2);
	buff[4] |= (static_cast<uint8_t>(flag_mosfet_temp)			<< 3);
	buff[4] |= (static_cast<uint8_t>(flag_duty_cycle_min)		<< 4);
	buff[4] |= (static_cast<uint8_t>(flag_duty_cycle_max)		<< 5);
	buff[4] |= (static_cast<uint8_t>(flag_local_mppt)			<< 6);
	buff[4] |= (static_cast<uint8_t>(flag_global_mppt)			<< 7);

	buff[6] = (static_cast<uint8_t>(mode));
	buff[7]= counter;
}

void Mpptx5::FromByteArray(uint8_t* buff)
{
	CANRXerr = buff[0];
	CANTXerr = buff[1];
	CANTXoverflow = buff[2];

	error_low_array_power 	= buff[3] & (1 << 0);
	error_mosfet_overheat 	= buff[3] & (1 << 1);
	error_battery_low 		= buff[3] & (1 << 2);
	error_battery_full 		= buff[3] & (1 << 3);
	error_12v_undervolt		= buff[3] & (1 << 4);
	error_hw_overcurrent 	= buff[3] & (1 << 6);
	error_hw_overvolt		= buff[3] & (1 << 7);

	flag_input_current_min	= buff[4] & (1 << 0);
	flag_input_current_max	= buff[4] & (1 << 1);
	flag_output_voltage_max	= buff[4] & (1 << 2);
	flag_mosfet_temp		= buff[4] & (1 << 3);
	flag_duty_cycle_min		= buff[4] & (1 << 4);
	flag_duty_cycle_max		= buff[4] & (1 << 5);
	flag_local_mppt			= buff[4] & (1 << 6);
	flag_global_mppt		= buff[4] & (1 << 7); 	
	
	mode = buff[5] & 0x1;
	counter = buff[7];
}

uint8_t Mpptx5::getCANRXerr() const{
	return CANRXerr;
}
uint8_t Mpptx5::getCANTXerr() const{
	return CANTXerr;
}
uint8_t Mpptx5::getCANTXoverflow() const{
	return CANTXoverflow;
}
uint8_t Mpptx5::getReserved() const{
	return reserved;
}
uint8_t Mpptx5::getCounter() const{
	return counter;
}

bool Mpptx5::getMode() const{
	return mode;
}
bool Mpptx5::isLowArrayPowerError() const{
	return error_low_array_power;
}
bool Mpptx5::isMosfetOverheatError() const{
	return error_mosfet_overheat;
}
bool Mpptx5::isBatteryLowError() const{
	return error_battery_low;
}
bool Mpptx5::isBatteryFullError() const{
	return error_battery_full;
}
bool Mpptx5::is12vUndervoltError() const{
	return error_12v_undervolt;
}
bool Mpptx5::isHWOvercurrentError() const{
	return error_hw_overcurrent;
}
bool Mpptx5::isHWOvervoltError() const{
	return error_hw_overvolt;
}

bool Mpptx5::isFlagInputCurrentMin() const{
	return flag_input_current_min;
}
bool Mpptx5::isFlagInputCurrentMax() const{
	return flag_input_current_max;
}
bool Mpptx5::isFlagOutputVoltageMax() const{
	return flag_output_voltage_max;
}
bool Mpptx5::isFlagMosfetTemp() const{
	return flag_mosfet_temp;
}
bool Mpptx5::isFlagDutyCycleMin() const{
	return flag_duty_cycle_min;
}
bool Mpptx5::isFlagDutyCycleMax() const{
	return flag_duty_cycle_max;
}
bool Mpptx5::isFlagLocalMppt() const{
	return flag_local_mppt;
}
bool Mpptx5::isFlagGlobalMppt() const{
	return flag_global_mppt;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx5::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX5_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX5_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX5_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx5::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("CANRXerr", getCANRXerr());
		http.addData("CANTXerr", getCANTXerr());
		http.addData("CANTXoverflow", getCANTXoverflow());

		http.addData("error_low_array_power", error_low_array_power);
		http.addData("error_mosfet_overheat", error_mosfet_overheat);
		http.addData("error_battery_low", error_battery_low);
		http.addData("error_battery_full", error_battery_full);
		http.addData("error_12v_undervolt", error_12v_undervolt);
		http.addData("error_hw_overcurrent", error_hw_overcurrent);
		http.addData("error_hw_overvolt", error_hw_overvolt);

		http.addData("flag_input_current_min", flag_input_current_min);
		http.addData("flag_input_current_max", flag_input_current_max);
		http.addData("flag_output_voltage_max", flag_output_voltage_max);
		http.addData("flag_mosfet_temp", flag_mosfet_temp);
		http.addData("flag_duty_cycle_min", flag_duty_cycle_min);
		http.addData("flag_duty_cycle_max", flag_duty_cycle_max);
		http.addData("flag_local_mppt", flag_local_mppt);
		http.addData("flag_global_mppt", flag_global_mppt);

		http.addData("mode", getMode());

		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx5", http.getParameters());
		http.flush();
	}
#endif

Mpptx6::Mpptx6(uint32_t can_id): // INCREMENT BY 4 FROM MPPTx0
	DataModule(can_id, 0, 8),
	battOutVolt(0),
	powerConnTemp(0) // unsure if i need to do this, orionBMS doesnt but steering does
{}

void Mpptx6::ToByteArray(uint8_t* buff) const
{
	f2b.f = battOutVolt;
	for (int i=0;i<=3;i++){
		buff[i] = f2b.s[i];
	}
	f2b.f = powerConnTemp;
	for (int i=4;i<=7;i++){
		buff[i] = f2b.s[i];
	}
}

void Mpptx6::FromByteArray(uint8_t* buff)
{
	for(int i=0;i<=3;i++){
		f2b.s[i] = buff[i];
	}

	battOutVolt = f2b.f;

	for(int i=4;i<=7;i++){
		f2b.s[i] = buff[i];
	}

	powerConnTemp = f2b.f;

}

float Mpptx6::getBattOutVolt() const{
	return battOutVolt;
}

float Mpptx6::getPowerConnTemp() const{
	return powerConnTemp;
}

#ifdef IS_TELEMETRY

	uint8_t Mpptx6::getMpptNo(){
		if (can_id_ == SolarGators::DataModuleInfo::MPPT0_RX6_MSG_ID){
			mpptNumber = MPPT0;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT1_RX6_MSG_ID){
			mpptNumber = MPPT1;
		} else if(can_id_ == SolarGators::DataModuleInfo::MPPT2_RX6_MSG_ID){
			mpptNumber = MPPT2;
		}
		return mpptNumber;
	}

	void Mpptx6::PostTelemetry(PythonScripts* scripts){
		PythonHttp http;
		http.init();
		http.addData("battOutVolt", getBattOutVolt());
		http.addData("powerConnTemp", getPowerConnTemp());
		http.addData("mpptNumber", getMpptNo());
		scripts->send("mppt/rx6", http.getParameters());
		http.flush();
	}
#endif

}
