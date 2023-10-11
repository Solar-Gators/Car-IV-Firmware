/*
 * Mitsuba.hpp
 *
 *  Created on: Jan 14, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_MITSUBA_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_MITSUBA_HPP_

#include <DataModule.hpp>

namespace SolarGators {
namespace DataModules {

class MitsubaRequest final : public DataModule
{
public:
  MitsubaRequest(uint32_t id);
  virtual ~MitsubaRequest();
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);
  static constexpr uint8_t Request_Size = 1;
  void SetRequests(bool frame0, bool frame1, bool frame2);
  void ClearRequests();
  void SetRequestAllFrames();
protected:
  bool requestFrame0;
  bool requestFrame1;
  bool requestFrame2;
};

class MitsubaRx0 final: public DataModule
{
public:
  MitsubaRx0(uint32_t can_id, uint16_t telem_id);
  virtual ~MitsubaRx0();
  // Getters
  float GetBatteryVoltage() const;
  uint16_t GetBatteryCurrent() const;
  enum BatteryCurrentDir {
    Discharging = 0,
    Charging
  };
  bool GetBatteryCurrentDir() const;
  uint16_t GetMotorCurrentPkAvg() const;
  uint16_t GetFetTemp() const;
  uint16_t GetMotorRPM() const;
  float GetPWMDuty() const;
  float GetLeadAngle() const;
  // Converter Functions
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);
  #ifdef IS_TELEMETRY
  void PostTelemetry(PythonScripts* scripts);
  #endif
  static constexpr uint8_t Rx0_Size = 8;
protected:
  uint16_t battVoltage;
  uint16_t battCurrent;
  bool     battCurrentDir;
  uint16_t motorCurrentPkAvg;
  uint8_t  FETtemp;
  uint16_t motorRPM;
  uint16_t PWMDuty;
  uint8_t  LeadAngle;
};

class MitsubaRx1 final: public DataModule
{
public:
  MitsubaRx1(uint32_t can_id, uint16_t telem_id);
  virtual ~MitsubaRx1();
  // Getters
  enum PowerMode {
    ECO,
    Power
  };
  bool GetPowerMode() const;
  enum MC_Mode {
    Current,
    PWM
  };
  bool GetMcMode() const;
  float GetAcceleratorPosition() const;
  float GetRegenVrPosition() const;
  uint8_t GetDigitSwitchPosition()const;
  float GetOutTargetVal() const;
  enum DriveAction {
    Reserved = 0,
    Forward = 1,
    Reverse = 2
  };
  uint8_t GetDriveActStat() const;
  bool GetRegenStat() const;
  // Converter Functions
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);
  #ifdef IS_TELEMETRY
  void PostTelemetry(PythonScripts* scripts);
  #endif
  static constexpr uint8_t Rx1_Size = 5;
protected:
  bool   powerMode;
  bool   MCmode;
  uint16_t AcceleratorPosition;
  uint16_t regenVRposition;
  uint8_t  digitSWposition;
  uint16_t outTargetVal;
  uint8_t  driveActStat;
  bool   regenStat;
};

class MitsubaRx2 final: public DataModule
{
public:
  MitsubaRx2(uint32_t can_id, uint16_t telem_id);
  virtual ~MitsubaRx2();
  // Getters
  bool GetAdSensorError() const;
  bool GetMotorSensorUError() const;
  bool GetMotorCurrSensorWError() const;
  bool GetFetThermError() const;
  bool GetBattVoltSensorError() const;
  bool GetBattCurrSensorError() const;
  bool GetBattCurrSensorAdjError() const;
  bool GetMotorCurrSensorAdjError() const;
  bool GetAccelPosError() const;
  bool GetContVoltSensorError() const;
  bool GetPowerSystemError() const;
  bool GetOverCurrError() const;
  bool GetOverVoltError() const;
  bool GetOverCurrLimit() const;
  bool GetMotorSystemError() const;
  bool GetMotorLock() const;
  bool GetHallSensorShort() const;
  bool GetHallSensorOpen() const;
  uint8_t GetOverHeatLevel() const;
  // Converter Functions
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);
  #ifdef IS_TELEMETRY
  void PostTelemetry(PythonScripts* scripts);
  #endif
  static constexpr uint8_t Rx2_Size = 5;
protected:
  bool adSensorError;
  bool motorCurrSensorUError;
  bool motorCurrSensorWError;
  bool fetThermError;
  bool battVoltSensorError;
  bool battCurrSensorError;
  bool battCurrSensorAdjError;
  bool motorCurrSensorAdjError;
  bool accelPosError;
  bool contVoltSensorError;
  bool powerSystemError;
  bool overCurrError;
  bool overVoltError;
  bool overCurrLimit;
  bool motorSystemError;
  bool motorLock;
  bool hallSensorShort;
  bool hallSensorOpen;
  uint8_t overHeatLevel;
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DATAMODULES_INC_MITSUBA_HPP_ */
