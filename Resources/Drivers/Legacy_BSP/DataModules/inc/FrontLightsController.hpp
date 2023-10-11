/*
 * FrontLightsController.hpp
 *
 *  Created on: Jun 13, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DATAMODULES_INC_FRONTLIGHTSCONTROLLER_HPP_
#define SOLARGATORSBSP_STM_DATAMODULES_INC_FRONTLIGHTSCONTROLLER_HPP_

#include <FrontLights.hpp>

namespace SolarGators {
namespace DataModules {

class FrontLightsController final: public FrontLights {
public:
  FrontLightsController();
  ~FrontLightsController();
  void SetThrottleVal(uint16_t);
  void SetBreaksVal(uint16_t);
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DATAMODULES_INC_FRONTLIGHTSCONTROLLER_HPP_ */
