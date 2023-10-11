/*
 * OrionBMS.cpp
 *
 *  Created on: Jan 14, 2022
 *      Author: John Carr
 */

#include "OrionBMS.hpp"

namespace SolarGators::DataModules
{
  // BMS Message 0
  OrionBMSRx0::OrionBMSRx0(uint32_t can_id, uint32_t telem_id):
        DataModule(can_id, telem_id, this->Size, 0, false)
  { }

  void OrionBMSRx0::ToByteArray(uint8_t* buff) const
  {
    buff[0] = low_cell_volt_ >> 8;
    buff[1] = (low_cell_volt_ & 0x00FF);
    buff[2] = high_cell_volt_ >> 8;
    buff[3] = (high_cell_volt_ & 0x00FF);
    buff[4] = avg_cell_volt_ >> 8;
    buff[5] = (avg_cell_volt_ & 0x00FF);
    buff[6] = pack_sum_volt_ >> 8;
    buff[7] = (pack_sum_volt_ & 0x00FF);
  }

  void OrionBMSRx0::FromByteArray(uint8_t* buff)
  {
    low_cell_volt_   = (static_cast<uint16_t>(buff[0]) << 8) | buff[1];
    high_cell_volt_  = (static_cast<uint16_t>(buff[2]) << 8) | buff[3];
    avg_cell_volt_   = (static_cast<uint16_t>(buff[4]) << 8) | buff[5];
    pack_sum_volt_   = (static_cast<uint16_t>(buff[6]) << 8) | buff[7];
  }

  float OrionBMSRx0::getAvgCellVolt() const {
    return avg_cell_volt_ * 1e-4;
  }

  float OrionBMSRx0::getHighCellVolt() const {
    return high_cell_volt_ * 1e-4;
  }

  float OrionBMSRx0::getLowCellVolt() const {
    return low_cell_volt_ * 1e-4;
  }

  float OrionBMSRx0::getPackSumVolt() const {
    return pack_sum_volt_ * 0.01;
  }
#ifdef IS_TELEMETRY
  void OrionBMSRx0::PostTelemetry(PythonScripts* scripts) {
    PythonHttp http;
    http.addData("low_cell_volt_", low_cell_volt_);
    http.addData("high_cell_volt_", high_cell_volt_);
    http.addData("avg_cell_volt_", avg_cell_volt_);
    http.addData("pack_sum_volt_", pack_sum_volt_);
    scripts->send("bms/rx0", http.getParameters());
    http.flush();
  }
#endif
  // BMS Message 1
  OrionBMSRx1::OrionBMSRx1(uint32_t can_id, uint32_t telem_id):
        DataModule(can_id, telem_id, this->Size, 0, false)
  { }

  void OrionBMSRx1::ToByteArray(uint8_t* buff) const
  {
    buff[0] = high_temp_;
    buff[1] = high_temp_id_;
    buff[2] = low_temp_;
    buff[3] = low_temp_id_;
    buff[4] = avg_temp_;
    buff[5] = internal_temp_;
    buff[6] = constant_val_ >> 8;
    buff[7] = (constant_val_ & 0x00FF);
  }

  void OrionBMSRx1::FromByteArray(uint8_t* buff)
  {
    high_temp_      = buff[0];
    high_temp_id_   = buff[1];
    low_temp_       = buff[2];
    low_temp_id_    = buff[3];
    avg_temp_       = buff[4];
    internal_temp_  = buff[5];
    constant_val_   = (static_cast<uint16_t>(buff[6]) << 8) | buff[7];
  }

  uint8_t OrionBMSRx1::getAvgTemp() const {
    return avg_temp_;
  }

  uint8_t OrionBMSRx1::getConstantVal() const {
    return constant_val_;
  }

  uint8_t OrionBMSRx1::getHighTemp() const {
    return high_temp_;
  }

  uint8_t OrionBMSRx1::getHighTempId() const {
    return high_temp_id_;
  }

  uint8_t OrionBMSRx1::getInternalTemp() const {
    return internal_temp_;
  }

  uint8_t OrionBMSRx1::getLowTemp() const {
    return low_temp_;
  }

  uint8_t OrionBMSRx1::getLowTempId() const {
    return low_temp_id_;
  }
#ifdef IS_TELEMETRY
  void OrionBMSRx1::PostTelemetry(PythonScripts* scripts) {
    PythonHttp http;
    http.addData("high_temp_", high_temp_);
    http.addData("high_temp_id_", high_temp_id_);
    http.addData("low_temp_", low_temp_);
    http.addData("low_temp_id_", low_temp_id_);
    http.addData("avg_temp_", avg_temp_);
    http.addData("internal_temp_", internal_temp_);
    http.addData("constant_val_", constant_val_);
    scripts->send("bms/rx1", http.getParameters());
    http.flush();
  }
#endif
  // BMS Message 2
  OrionBMSRx2::OrionBMSRx2(uint32_t can_id, uint32_t telem_id):
        DataModule(can_id, telem_id, this->Size, 0, false)
  { }

  void OrionBMSRx2::ToByteArray(uint8_t* buff) const
  {
    buff[0] = pack_dcl_ >> 8;
    buff[1] = (pack_dcl_ & 0x00FF);
    buff[2] = pack_ccl_ >> 8;
    buff[3] = (pack_ccl_ & 0x00FF);
    buff[4] = pack_current_ >> 8;
    buff[5] = (pack_current_ & 0x00FF);
    buff[6] = constant_val_ >> 8;
    buff[7] = (constant_val_ & 0x00FF);
  }

  void OrionBMSRx2::FromByteArray(uint8_t* buff)
  {
    pack_dcl_     = (static_cast<uint16_t>(buff[0]) << 8) | buff[1];
    pack_ccl_     = (static_cast<uint16_t>(buff[2]) << 8) | buff[3];
    pack_current_ = (static_cast<uint16_t>(buff[4]) << 8) | buff[5];
    constant_val_ = (static_cast<uint16_t>(buff[6]) << 8) | buff[7];
  }

  uint16_t OrionBMSRx2::getConstantVal() const {
    return constant_val_;
  }

  uint16_t OrionBMSRx2::getPackCcl() const {
    return pack_ccl_;
  }

  float OrionBMSRx2::getPackCurrent() const {
    return pack_current_ * 0.1;
  }

  uint16_t OrionBMSRx2::getPackDcl() const {
    return pack_dcl_;
  }
#ifdef IS_TELEMETRY
  void OrionBMSRx2::PostTelemetry(PythonScripts* scripts) {
    PythonHttp http;
    http.addData("pack_dcl_", pack_dcl_);
    http.addData("pack_ccl_", pack_ccl_);
    http.addData("pack_current_", pack_current_);
    http.addData("constant_val_", constant_val_);
    scripts->send("bms/rx2", http.getParameters());
    http.flush();
  }
#endif
  // BMS Message 3
  OrionBMSRx3::OrionBMSRx3(uint32_t can_id, uint32_t telem_id):
        DataModule(can_id, telem_id, this->Size, 0, false)
  { }

  void OrionBMSRx3::ToByteArray(uint8_t* buff) const
    {
      buff[0] = low_cell_res_ >> 8;
      buff[1] = (low_cell_res_ & 0x00FF);
      buff[2] = high_cell_res_ >> 8;
      buff[3] = (high_cell_res_ & 0x00FF);
      buff[4] = pack_res_ >> 8;
      buff[5] = (pack_res_ & 0x00FF);
    }

    void OrionBMSRx3::FromByteArray(uint8_t* buff)
    {
      low_cell_res_  = (static_cast<uint16_t>(buff[0]) << 8) | buff[1];
      high_cell_res_  = (static_cast<uint16_t>(buff[2]) << 8) | buff[3];
      pack_res_ = (static_cast<uint16_t>(buff[4]) << 8) | buff[5];
    }

  float OrionBMSRx3::getHighCellRes() const {
    return high_cell_res_ * 0.01;
  }

  float OrionBMSRx3::getLowCellRes() const {
    return low_cell_res_ * 0.01;
  }

  float OrionBMSRx3::getPackRes() const {
    return pack_res_ * 0.001;
  }
#ifdef IS_TELEMETRY
  void OrionBMSRx3::PostTelemetry(PythonScripts* scripts) {
    PythonHttp http;
    http.addData("low_cell_res_", low_cell_res_);
    http.addData("high_cell_res_", high_cell_res_);
    http.addData("pack_res_", pack_res_);
    scripts->send("bms/rx3", http.getParameters());
    http.flush();
  }
#endif
  // BMS Message 4
  OrionBMSRx4::OrionBMSRx4(uint32_t can_id, uint32_t telem_id):
        DataModule(can_id, telem_id, this->Size, 0, false)
  { }

  void OrionBMSRx4::ToByteArray(uint8_t* buff) const
  {
    buff[0]  = (static_cast<uint8_t>(internal_cell_communication_fault_)   << 0);
    buff[0] |= (static_cast<uint8_t>(cell_balancing_stuck_off_fault_)      << 1);
    buff[0] |= (static_cast<uint8_t>(weak_cell_fault_)                     << 2);
    buff[0] |= (static_cast<uint8_t>(low_cell_voltage_fault_)              << 3);
    buff[0] |= (static_cast<uint8_t>(cell_open_wiring_fault_)              << 4);
    buff[0] |= (static_cast<uint8_t>(current_sensor_fault_)                << 5);
    buff[0] |= (static_cast<uint8_t>(cell_voltage_over_5v_fault_)          << 6);
    buff[0] |= (static_cast<uint8_t>(cell_bank_fault_)                     << 7);

    buff[1]  = (static_cast<uint8_t>(weak_pack_fault_)                     << 0);
    buff[1] |= (static_cast<uint8_t>(fan_monitor_fault_)                   << 1);
    buff[1] |= (static_cast<uint8_t>(thermistor_fault_)                    << 2);
    buff[1] |= (static_cast<uint8_t>(can_communication_fault_)             << 3);
    buff[1] |= (static_cast<uint8_t>(redundant_power_supply_fault_)        << 4);
    buff[1] |= (static_cast<uint8_t>(high_voltage_isolation_fault_)        << 5);
    buff[1] |= (static_cast<uint8_t>(invalid_input_supply_voltage_fault_)  << 6);
    buff[1] |= (static_cast<uint8_t>(chargeenable_relay_fault_)            << 7);

    buff[2]  = (static_cast<uint8_t>(dischargeenable_relay_fault_)         << 0);
    buff[2] |= (static_cast<uint8_t>(charger_safety_relay_fault_)          << 1);
    buff[2] |= (static_cast<uint8_t>(internal_hardware_fault_)             << 2);
    buff[2] |= (static_cast<uint8_t>(internal_heatsink_thermistor_fault_)  << 3);
    buff[2] |= (static_cast<uint8_t>(internal_logic_fault_)                << 4);
    buff[2] |= (static_cast<uint8_t>(highest_cell_voltage_too_high_fault_) << 5);
    buff[2] |= (static_cast<uint8_t>(lowest_cell_voltage_too_low_fault_)   << 6);
    buff[2] |= (static_cast<uint8_t>(pack_too_hot_fault_)                  << 7);

    buff[3]  = pack_soc_;
  }

  void OrionBMSRx4::FromByteArray(uint8_t* buff)
  {
    internal_cell_communication_fault_  = buff[0] & (1 << 0);
    cell_balancing_stuck_off_fault_     = buff[0] & (1 << 1);
    weak_cell_fault_                    = buff[0] & (1 << 2);
    low_cell_voltage_fault_             = buff[0] & (1 << 3);
    cell_open_wiring_fault_             = buff[0] & (1 << 4);
    current_sensor_fault_               = buff[0] & (1 << 5);
    cell_voltage_over_5v_fault_         = buff[0] & (1 << 6);
    cell_bank_fault_                    = buff[0] & (1 << 7);

    weak_pack_fault_                    = buff[1] & (1 << 0);
    fan_monitor_fault_                  = buff[1] & (1 << 1);
    thermistor_fault_                   = buff[1] & (1 << 2);
    can_communication_fault_            = buff[1] & (1 << 3);
    redundant_power_supply_fault_       = buff[1] & (1 << 4);
    high_voltage_isolation_fault_       = buff[1] & (1 << 5);
    invalid_input_supply_voltage_fault_ = buff[1] & (1 << 6);
    chargeenable_relay_fault_           = buff[1] & (1 << 7);

    dischargeenable_relay_fault_        = buff[2] & (1 << 0);
    charger_safety_relay_fault_         = buff[2] & (1 << 1);
    internal_hardware_fault_            = buff[2] & (1 << 2);
    internal_heatsink_thermistor_fault_ = buff[2] & (1 << 3);
    internal_logic_fault_               = buff[2] & (1 << 4);
    highest_cell_voltage_too_high_fault_= buff[2] & (1 << 5);
    lowest_cell_voltage_too_low_fault_  = buff[2] & (1 << 6);
    pack_too_hot_fault_                 = buff[2] & (1 << 7);

    pack_soc_ = buff[3];
  }

  bool OrionBMSRx4::isCanCommunicationFault() const {
    return can_communication_fault_;
  }

  bool OrionBMSRx4::isCellBalancingStuckOffFault() const {
    return cell_balancing_stuck_off_fault_;
  }

  bool OrionBMSRx4::isCellBankFault() const {
    return cell_bank_fault_;
  }

  bool OrionBMSRx4::isCellOpenWiringFault() const {
    return cell_open_wiring_fault_;
  }

  bool OrionBMSRx4::isCellVoltageOver5vFault() const {
    return cell_voltage_over_5v_fault_;
  }

  bool OrionBMSRx4::isChargeenableRelayFault() const {
    return chargeenable_relay_fault_;
  }

  bool OrionBMSRx4::isChargerSafetyRelayFault() const {
    return charger_safety_relay_fault_;
  }

  bool OrionBMSRx4::isCurrentSensorFault() const {
    return current_sensor_fault_;
  }

  bool OrionBMSRx4::isDischargeenableRelayFault() const {
    return dischargeenable_relay_fault_;
  }

  bool OrionBMSRx4::isFanMonitorFault() const {
    return fan_monitor_fault_;
  }

  bool OrionBMSRx4::isHighVoltageIsolationFault() const {
    return high_voltage_isolation_fault_;
  }

  bool OrionBMSRx4::isHighestCellVoltageTooHighFault() const {
    return highest_cell_voltage_too_high_fault_;
  }

  bool OrionBMSRx4::isInternalCellCommunicationFault() const {
    return internal_cell_communication_fault_;
  }

  bool OrionBMSRx4::isInternalHardwareFault() const {
    return internal_hardware_fault_;
  }

  bool OrionBMSRx4::isInternalHeatsinkThermistorFault() const {
    return internal_heatsink_thermistor_fault_;
  }

  bool OrionBMSRx4::isInternalLogicFault() const {
    return internal_logic_fault_;
  }

  bool OrionBMSRx4::isInvalidInputSupplyVoltageFault() const {
    return invalid_input_supply_voltage_fault_;
  }

  bool OrionBMSRx4::isLowCellVoltageFault() const {
    return low_cell_voltage_fault_;
  }

  bool OrionBMSRx4::isLowestCellVoltageTooLowFault() const {
    return lowest_cell_voltage_too_low_fault_;
  }

  bool OrionBMSRx4::isPackTooHotFault() const {
    return pack_too_hot_fault_;
  }

  bool OrionBMSRx4::isRedundantPowerSupplyFault() const {
    return redundant_power_supply_fault_;
  }

  bool OrionBMSRx4::isThermistorFault() const {
    return thermistor_fault_;
  }

  bool OrionBMSRx4::isWeakCellFault() const {
    return weak_cell_fault_;
  }

  bool OrionBMSRx4::isWeakPackFault() const {
    return weak_pack_fault_;
  }

  float OrionBMSRx4::getPackSoc() const {
    return pack_soc_ * 0.5;
  }
#ifdef IS_TELEMETRY
  void OrionBMSRx4::PostTelemetry(PythonScripts* scripts) {
    PythonHttp http;
    http.addData("internal_cell_communication_fault_", internal_cell_communication_fault_);
    http.addData("cell_balancing_stuck_off_fault_", cell_balancing_stuck_off_fault_);
    http.addData("weak_cell_fault_", weak_cell_fault_);
    http.addData("low_cell_voltage_fault_", low_cell_voltage_fault_);
    http.addData("cell_open_wiring_fault_", cell_open_wiring_fault_);
    http.addData("current_sensor_fault_", current_sensor_fault_);
    http.addData("cell_voltage_over_5v_fault_", cell_voltage_over_5v_fault_);
    http.addData("cell_bank_fault_", cell_bank_fault_);
    http.addData("weak_pack_fault_", weak_pack_fault_);
    http.addData("fan_monitor_fault_", fan_monitor_fault_);
    http.addData("thermistor_fault_", thermistor_fault_);
    http.addData("can_communication_fault_", can_communication_fault_);
    http.addData("redundant_power_supply_fault_", redundant_power_supply_fault_);
    http.addData("high_voltage_isolation_fault_", high_voltage_isolation_fault_);
    http.addData("invalid_input_supply_voltage_fault_", invalid_input_supply_voltage_fault_);
    http.addData("chargeenable_relay_fault_", chargeenable_relay_fault_);
    http.addData("dischargeenable_relay_fault_", dischargeenable_relay_fault_);
    http.addData("charger_safety_relay_fault_", charger_safety_relay_fault_);
    http.addData("internal_hardware_fault_", internal_hardware_fault_);
    http.addData("internal_heatsink_thermistor_fault_", internal_heatsink_thermistor_fault_);
    http.addData("internal_logic_fault_", internal_logic_fault_);
    http.addData("highest_cell_voltage_too_high_fault_", highest_cell_voltage_too_high_fault_);
    http.addData("lowest_cell_voltage_too_low_fault_", lowest_cell_voltage_too_low_fault_);
    http.addData("pack_too_hot_fault_", pack_too_hot_fault_);
    http.addData("pack_soc_", pack_soc_);
    scripts->send("bms/rx4", http.getParameters());
    http.flush();
  }
#endif
  // BMS Message 5
  OrionBMSRx5::OrionBMSRx5(uint32_t can_id, uint32_t telem_id):
        DataModule(can_id, telem_id, this->Size, 0, false)
  { }

  void OrionBMSRx5::ToByteArray(uint8_t* buff) const
  {
    buff[0] = max_pack_dcl_ >> 8;
    buff[1] = (max_pack_dcl_ & 0x00FF);
    buff[2] = max_pack_ccl_ >> 8;
    buff[3] = (max_pack_ccl_ & 0x00FF);
    buff[4] = max_pack_volt_ >> 8;
    buff[5] = (max_pack_volt_ & 0x00FF);
    buff[6] = min_pack_volt_ >> 8;
    buff[7] = (min_pack_volt_ & 0x00FF);
  }

  void OrionBMSRx5::FromByteArray(uint8_t* buff)
  {
    max_pack_dcl_  = (static_cast<uint16_t>(buff[0]) << 8) | buff[1];
    max_pack_ccl_  = (static_cast<uint16_t>(buff[2]) << 8) | buff[3];
    max_pack_volt_ = (static_cast<uint16_t>(buff[4]) << 8) | buff[5];
    min_pack_volt_ = (static_cast<uint16_t>(buff[6]) << 8) | buff[7];
  }

  uint16_t OrionBMSRx5::getMaxPackCcl() const {
    return max_pack_ccl_;
  }

  uint16_t OrionBMSRx5::getMaxPackDcl() const {
    return max_pack_dcl_;
  }

  float OrionBMSRx5::getMaxPackVolt() const {
    return max_pack_volt_ * 0.1;
  }

  float OrionBMSRx5::getMinPackVolt() const {
    return min_pack_volt_ * 0.1;
  }
#ifdef IS_TELEMETRY
  void OrionBMSRx5::PostTelemetry(PythonScripts* scripts) {
    PythonHttp http;
    http.addData("max_pack_dcl_", max_pack_dcl_);
    http.addData("max_pack_ccl_", max_pack_ccl_);
    http.addData("max_pack_volt_", max_pack_volt_);
    http.addData("min_pack_volt_", min_pack_volt_);
    scripts->send("bms/rx5", http.getParameters());
    http.flush();
  }
#endif
}
