/*
 * Steering.cpp
 *
 *  Created on: Oct 28, 2021
 *      Author: John Carr
 */

#include "Steering.hpp"
#include <string.h>

namespace {
  static constexpr uint32_t ID = 10;
  static constexpr uint32_t SIZE = 3;
}
namespace SolarGators::DataModules
{
  Steering::Steering():
    DataModule(ID, 0, SIZE),
    left_turn_(false),
    right_turn_(false),
    hazards_(false),
    bps_fault_(false),
    cruise_enable_(false),
    eco_enable_(true),
    headlights_(false),
    horn_(false),
    reverse_(false),
	regen_(false),
    cruise_speed_(Min_Cruise_Speed_)
  {}
  Steering::~Steering()
  {}
  bool Steering::GetLeftTurnStatus() const
  {
    return left_turn_;
  }
  bool Steering::GetRightTurnStatus() const
  {
    return right_turn_;
  }
  bool Steering::GetHazardsStatus() const
  {
    return hazards_;
  }
  bool Steering::GetBpFaultStatus() const
  {
    return bps_fault_;
  }
  bool Steering::GetCruiseEnabledStatus() const
  {
    return cruise_enable_;
  }
  bool Steering::GetEcoEnabledStatus() const
  {
    return eco_enable_;
  }
  bool Steering::GetHeadlightsStatus() const
  {
    return headlights_;
  }
  bool Steering::GetHornStatus() const
  {
    return horn_;
  }
  bool Steering::GetReverseStatus() const
  {
    return reverse_;
  }
  uint8_t Steering::GetCruiseSpeed() const
  {
    return cruise_speed_;
  }
  uint8_t Steering::GetRegen() const
  {
	  return regen_;
  }
  void Steering::ToByteArray(uint8_t* buff) const
  {
    memset(buff, 0, sizeof(buff));
    buff[0] |= (static_cast<uint8_t>(left_turn_)     << 0);
    buff[0] |= (static_cast<uint8_t>(right_turn_)    << 1);
    buff[0] |= (static_cast<uint8_t>(hazards_)       << 2);
    buff[0] |= (static_cast<uint8_t>(bps_fault_)     << 3);
    buff[0] |= (static_cast<uint8_t>(cruise_enable_) << 4);
    buff[0] |= (static_cast<uint8_t>(eco_enable_)    << 5);
    buff[0] |= (static_cast<uint8_t>(headlights_)    << 6);
    buff[0] |= (static_cast<uint8_t>(horn_)          << 7);
    buff[1] |= (static_cast<uint8_t>(reverse_)       << 0);
    buff[1] |= regen_ 								 << 1;
    buff[2] |= cruise_speed_;
  }
  void Steering::FromByteArray(uint8_t* buff)
  {
    left_turn_      = buff[0] & (1 << 0);
    right_turn_     = buff[0] & (1 << 1);
    hazards_        = buff[0] & (1 << 2);
    bps_fault_      = buff[0] & (1 << 3);
    cruise_enable_  = buff[0] & (1 << 4);
    eco_enable_     = buff[0] & (1 << 5);
    headlights_     = buff[0] & (1 << 6);
    horn_           = buff[0] & (1 << 7);
    reverse_        = buff[1] & (1 << 0);
    regen_			= (buff[1] & (3 << 1)) >> 1;
    cruise_speed_   = buff[2];
  }
#ifdef IS_TELEMETRY
  void Steering::PostTelemetry(PythonScripts* scripts) {

  }
#endif
}
