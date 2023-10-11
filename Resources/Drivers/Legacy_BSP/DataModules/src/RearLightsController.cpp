/*
 * RearLightsController.cpp
 *
 *  Created on: Jun 17, 2022
 *      Author: John Carr
 */

#include <RearLightsController.hpp>

namespace SolarGators {
namespace DataModules {

RearLightsController::RearLightsController()
{ }

RearLightsController::~RearLightsController()
{ }

void RearLightsController::SetBreakPressed()
{
  break_ = true;
}

void RearLightsController::SetBreakReleased()
{
  break_ = false;
}

} /* namespace DataModules */
} /* namespace SolarGators */
