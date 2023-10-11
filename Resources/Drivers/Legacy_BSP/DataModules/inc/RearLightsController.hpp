/*
 * RearLightsController.hpp
 *
 *  Created on: Jun 17, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DATAMODULES_INC_REARLIGHTSCONTROLLER_HPP_
#define SOLARGATORSBSP_STM_DATAMODULES_INC_REARLIGHTSCONTROLLER_HPP_

#include "RearLights.hpp"

namespace SolarGators {
namespace DataModules {

class RearLightsController final: public RearLights {
public:
  RearLightsController();
  virtual ~RearLightsController();
  void SetBreakPressed();
  void SetBreakReleased();
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DATAMODULES_INC_REARLIGHTSCONTROLLER_HPP_ */
