/*
 * Proton1.hpp
 *
 *  Created on: Jan 14, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_PROTON1_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_PROTON1_HPP_

#include <DataModule.hpp>

namespace SolarGators {
namespace DataModules {

class Proton1 final : public DataModule {
public:
  Proton1(uint32_t id);
  virtual ~Proton1();
  // Getters
  float getArrayVoltage() const;
  float getArrayCurrent() const;
  float getBatteryVoltage() const;
  float getMpptTemperature() const;
  // Converter Functions
  void ToByteArray(uint8_t* buff) const;
  void FromByteArray(uint8_t* buff);
  #ifdef IS_TELEMETRY
void PostTelemetry(PythonScripts* scripts);
#endif

protected:
  float arrayVoltage;
  float arrayCurrent;
  float batteryVoltage;
  float mpptTemperature;
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DATAMODULES_INC_PROTON1_HPP_ */
