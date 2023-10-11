/*
 * Steering.hpp
 *
 *  Created on: Oct 28, 2021
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_STEERING_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_STEERING_HPP_

#include <DataModule.hpp>
#include <cstdint>

namespace SolarGators::DataModules
{
  class Steering : public DataModule {
  public:
    Steering();
    ~Steering();
    bool GetLeftTurnStatus() const;
    bool GetRightTurnStatus() const;
    bool GetHazardsStatus() const;
    bool GetBpFaultStatus() const;
    bool GetCruiseEnabledStatus() const;
    bool GetEcoEnabledStatus() const;
    bool GetHeadlightsStatus() const;
    bool GetHornStatus() const;
    bool GetReverseStatus() const;
    uint8_t GetCruiseSpeed() const;
    uint8_t GetRegen() const;
    void ToByteArray(uint8_t* buff) const;
    void FromByteArray(uint8_t* buff);
    #ifdef IS_TELEMETRY
void PostTelemetry(PythonScripts* scripts);
#endif
    static constexpr uint8_t Max_Cruise_Speed_ = 220;
    static constexpr uint8_t Min_Cruise_Speed_ = 0;
  protected:
    bool left_turn_;
    bool right_turn_;
    bool hazards_;
    bool bps_fault_;
    bool cruise_enable_;
    bool eco_enable_;
    bool headlights_;
    bool horn_;
    bool reverse_;
    uint8_t cruise_speed_;
    uint8_t regen_;
  };
}

#endif /* SOLARGATORSBSP_DATAMODULES_INC_STEERING_HPP_ */
