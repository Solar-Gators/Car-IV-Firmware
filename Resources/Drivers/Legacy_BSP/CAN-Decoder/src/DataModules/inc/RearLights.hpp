/*
 * RearLights.hpp
 *
 *  Created on: Jun 17, 2022
 *      Author: John Carr, Yash Bhat
 */

#ifndef SOLARGATORSBSP_STM_DATAMODULES_INC_REARLIGHTS_HPP_
#define SOLARGATORSBSP_STM_DATAMODULES_INC_REARLIGHTS_HPP_

#include "DataModule.hpp"

namespace SolarGators {
namespace DataModules {

class RearLights: public DataModule {
public:
  RearLights();
  virtual ~RearLights();
  bool isBreakPressed() const;
  void setKillSwStatus(bool status);
  bool getKillSwStatus() const;
  void setContactorStatus(bool status);
  bool getContactorStatus() const;
  void doATrip();
  // CAN Functions
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);


protected:
  bool break_;
  bool killsw_;
  bool contactor_status_;
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DATAMODULES_INC_REARLIGHTS_HPP_ */
