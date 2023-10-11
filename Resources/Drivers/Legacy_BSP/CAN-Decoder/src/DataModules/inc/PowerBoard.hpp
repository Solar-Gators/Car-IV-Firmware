/*
 * PowerBoard.hpp
 *
 *  Created on: Apr 21, 2023
 *      Author: Taylor Gerke
 */

#ifndef SOLARGATORSBSP_STM_CAN_DECODER_SRC_DATAMODULES_INC_POWERBOARD_HPP_
#define SOLARGATORSBSP_STM_CAN_DECODER_SRC_DATAMODULES_INC_POWERBOARD_HPP_

#include <DataModule.hpp>

namespace SolarGators {
namespace DataModules {

class PowerBoard: public DataModule {
public:
  PowerBoard();
  ~PowerBoard();
  float GetSupBatVoltage();
  float GetSupBatPower();
  float GetMainBatPower();
  uint8_t GetPowerSource();
  // CAN Functions
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);

protected:
  uint16_t SupBatVoltage_; //has been converted from float
  uint16_t SupBatPower_;   //has been converted from float
  uint16_t MainBatPower_;  //has been converted from float
  uint8_t PowerSource_;    //bool for sup/main, 0 for sup, 1 for main
};

} /* namespace DataModules */
} /* namespace SolarGators */



#endif /* SOLARGATORSBSP_STM_CAN_DECODER_SRC_DATAMODULES_INC_POWERBOARD_HPP_ */
