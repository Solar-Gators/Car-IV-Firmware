/*
 * DataModule.h
 *
 *  Created on: Oct 29, 2021
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_DATAMODULE_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_DATAMODULE_HPP_

#include <cstdint>

#ifdef IS_TELEMETRY
#include <PythonHttp.hpp>
#include <PythonScripts.hpp>
#endif

#ifndef IS_TELEMETRY
#include <cmsis_os.h>
#endif

namespace SolarGators {
namespace DataModules {

class DataModule {
public:
  DataModule(uint32_t can_id, uint16_t telem_id, uint32_t size, uint16_t instance_id = 0, bool is_ext_id = false, bool is_rtr = false):
    can_id_(can_id), telem_id_(telem_id), size_(size), instance_id_(instance_id), is_ext_id_(is_ext_id), is_rtr_(is_rtr)
    {
      #ifndef IS_TELEMETRY
      mutex_id_ = osMutexNew(&mutex_attributes_);
      #endif
    };
  virtual ~DataModule() {};
  // All data modules must define this so that we can parse the can messages
  virtual void ToByteArray(uint8_t* buff) const = 0;
  virtual void FromByteArray(uint8_t* buff) = 0;
  #ifdef IS_TELEMETRY
  virtual void PostTelemetry(PythonScripts* scripts) = 0;
  #endif
  // The can bus ID for the data module
  const uint32_t can_id_;
  // ID for transmitting the data module to the pit
  const uint16_t telem_id_;
  // Amount of data in bytes
  const uint32_t size_;
  // Instance ID
  const uint16_t instance_id_;
  // If the can bus ID is extended
  const bool is_ext_id_;
  // If the can message is RTR
  const bool is_rtr_;

  #ifndef IS_TELEMETRY
  // Mutex for the data module
  osMutexId_t mutex_id_;
  StaticSemaphore_t mutex_control_block_;
  const osMutexAttr_t mutex_attributes_ = {
    .name = "DMM",
    .attr_bits = osMutexRecursive,
    .cb_mem = &mutex_control_block_,
    .cb_size = sizeof(mutex_control_block_),
  };
  #endif
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DATAMODULES_INC_DATAMODULE_HPP_ */
