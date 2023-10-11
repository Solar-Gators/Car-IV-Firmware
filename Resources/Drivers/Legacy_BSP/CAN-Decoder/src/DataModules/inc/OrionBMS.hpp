/*
 * OrionBMS.hpp
 *
 *  Created on: Jan 14, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_ORIONBMS_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_ORIONBMS_HPP_

#include <DataModule.hpp>

namespace SolarGators::DataModules
{
  class OrionBMSRx0 final: public DataModule
  {
  public:
    OrionBMSRx0(uint32_t can_id, uint32_t telem_id);
    ~OrionBMSRx0() {};

    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
    void PostTelemetry(PythonScripts* scripts);
    #endif

    float getAvgCellVolt() const;
    float getHighCellVolt() const;
    float getLowCellVolt() const;
    float getPackSumVolt() const;

    static constexpr uint8_t Size = 8;
  protected:
    uint16_t low_cell_volt_;
    uint16_t high_cell_volt_;
    uint16_t avg_cell_volt_;
    uint16_t pack_sum_volt_;
  };

  class OrionBMSRx1 final: public DataModule
  {
  public:
    OrionBMSRx1(uint32_t can_id, uint32_t telem_id);
    ~OrionBMSRx1() {};

    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
    void PostTelemetry(PythonScripts* scripts);
    #endif

    uint8_t getAvgTemp() const;
    uint8_t getConstantVal() const;
    uint8_t getHighTemp() const;
    uint8_t getHighTempId() const;
    uint8_t getInternalTemp() const;
    uint8_t getLowTemp() const;
    uint8_t getLowTempId() const;

    static constexpr uint8_t Size = 8;
  protected:
    uint8_t high_temp_;
    uint8_t high_temp_id_;
    uint8_t low_temp_;
    uint8_t low_temp_id_;
    uint8_t avg_temp_;
    uint8_t internal_temp_;
    uint8_t constant_val_;
  };

  class OrionBMSRx2 final: public DataModule
  {
  public:
    OrionBMSRx2(uint32_t can_id, uint32_t telem_id);
    ~OrionBMSRx2() {};

    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
    void PostTelemetry(PythonScripts* scripts);
    #endif

    uint16_t getConstantVal() const;
    uint16_t getPackCcl() const;
    float getPackCurrent() const;
    uint16_t getPackDcl() const;

    static constexpr uint8_t Size = 8;
  protected:
    uint16_t pack_dcl_;
    uint16_t pack_ccl_;
    int16_t pack_current_;
    uint16_t constant_val_;
  };

  class OrionBMSRx3 final: public DataModule
  {
  public:
    OrionBMSRx3(uint32_t can_id, uint32_t telem_id);
    ~OrionBMSRx3() {};

    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
    void PostTelemetry(PythonScripts* scripts);
    #endif

    float getHighCellRes() const;
    float getLowCellRes() const;
    float getPackRes() const;

    static constexpr uint8_t Size = 6;
  protected:
    uint16_t low_cell_res_;
    uint16_t high_cell_res_;
    uint16_t pack_res_;
  };

  class OrionBMSRx4 final: public DataModule
  {
  public:
    OrionBMSRx4(uint32_t can_id, uint32_t telem_id);
    ~OrionBMSRx4() {};

    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
    void PostTelemetry(PythonScripts* scripts);
    #endif

    bool isCanCommunicationFault() const;
    bool isCellBalancingStuckOffFault() const;
    bool isCellBankFault() const;
    bool isCellOpenWiringFault() const;
    bool isCellVoltageOver5vFault() const;
    bool isChargeenableRelayFault() const;
    bool isChargerSafetyRelayFault() const;
    bool isCurrentSensorFault() const;
    bool isDischargeenableRelayFault() const;
    bool isFanMonitorFault() const;
    bool isHighVoltageIsolationFault() const;
    bool isHighestCellVoltageTooHighFault() const;
    bool isInternalCellCommunicationFault() const;
    bool isInternalHardwareFault() const;
    bool isInternalHeatsinkThermistorFault() const;
    bool isInternalLogicFault() const;
    bool isInvalidInputSupplyVoltageFault() const;
    bool isLowCellVoltageFault() const;
    bool isLowestCellVoltageTooLowFault() const;
    bool isPackTooHotFault() const;
    bool isRedundantPowerSupplyFault() const;
    bool isThermistorFault() const;
    bool isWeakCellFault() const;
    bool isWeakPackFault() const;
    float getPackSoc() const;

    static constexpr uint8_t Size = 4;
  protected:
    bool internal_cell_communication_fault_;
    bool cell_balancing_stuck_off_fault_;
    bool weak_cell_fault_;
    bool low_cell_voltage_fault_;
    bool cell_open_wiring_fault_;
    bool current_sensor_fault_;
    bool cell_voltage_over_5v_fault_;
    bool cell_bank_fault_;
    bool weak_pack_fault_;
    bool fan_monitor_fault_;
    bool thermistor_fault_;
    bool can_communication_fault_;
    bool redundant_power_supply_fault_;
    bool high_voltage_isolation_fault_;
    bool invalid_input_supply_voltage_fault_;
    bool chargeenable_relay_fault_;
    bool dischargeenable_relay_fault_;
    bool charger_safety_relay_fault_;
    bool internal_hardware_fault_;
    bool internal_heatsink_thermistor_fault_;
    bool internal_logic_fault_;
    bool highest_cell_voltage_too_high_fault_;
    bool lowest_cell_voltage_too_low_fault_;
    bool pack_too_hot_fault_;
    uint8_t pack_soc_;
  };

  class OrionBMSRx5 final: public DataModule
  {
  public:
    OrionBMSRx5(uint32_t can_id, uint32_t telem_id);
    ~OrionBMSRx5() {};

    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
    void PostTelemetry(PythonScripts* scripts);
    #endif

    uint16_t getMaxPackCcl() const;
    uint16_t getMaxPackDcl() const;
    float getMaxPackVolt() const;
    float getMinPackVolt() const;

    static constexpr uint8_t Size = 8;
  protected:
    uint16_t max_pack_dcl_;
    uint16_t max_pack_ccl_;
    uint16_t max_pack_volt_;
    uint16_t min_pack_volt_;
  };

}

#endif /* SOLARGATORSBSP_DATAMODULES_INC_ORIONBMS_HPP_ */
