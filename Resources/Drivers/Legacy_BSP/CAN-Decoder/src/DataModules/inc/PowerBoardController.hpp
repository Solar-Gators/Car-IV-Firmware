/*
 * FrontLightsController.hpp
 *
 *  Created on: Jun 13, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DATAMODULES_INC_POWERBOARDCONTROLLER_HPP_
#define SOLARGATORSBSP_STM_DATAMODULES_INC_POWERBOARDCONTROLLER_HPP_
#include <DataModule.hpp>

namespace SolarGators {
namespace DataModules {

class PowerBoardController final: public DataModule {
public:
  PowerBoardController(uint32_t can_id, uint16_t telem_id);
  ~PowerBoardController();
  void SetSupBatVoltage(uint16_t);
  void SetSupBatPower(uint16_t);
  void SetMainBatPower(uint16_t);
  void SetPowerSource(uint8_t);
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);
  static constexpr uint8_t Size = 8;

protected:
  uint16_t SupBatVoltage; //has been converted from float
  uint16_t SupBatPower;   //has been converted from float
  uint16_t MainBatPower;  //has been converted from float
  uint8_t PowerSource;    //bool for sup/main, 0 for sup, 1 for main
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DATAMODULES_INC_FRONTLIGHTSCONTROLLER_HPP_ */
