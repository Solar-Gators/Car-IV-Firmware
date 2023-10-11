/*
 * CAN.hpp
 *
 *  Created on: Oct 29, 2021
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DRIVERS_INC_CAN_HPP_
#define SOLARGATORSBSP_DRIVERS_INC_CAN_HPP_

#include <cmsis_os.h>
#include "main.h"
#include <DataModule.hpp>
#include "etl/map.h"

namespace SolarGators {
namespace Drivers {

class CANDriver {
public:
  CANDriver(CAN_HandleTypeDef* hcan, uint32_t rx_fifo_num_);
  void Init();
  virtual ~CANDriver();
  void Send(SolarGators::DataModules::DataModule* data);
  void HandleReceive();
  void SetRxFlag();
  bool AddRxModule(DataModules::DataModule* module);
  bool RemoveRxModule(uint32_t module_id);
  static constexpr uint8_t MAX_DATA_SIZE = 8;     // Maximum data size in bytes
private:
  ::etl::map<uint32_t, SolarGators::DataModules::DataModule*, 15> modules_;
  CAN_HandleTypeDef* hcan_;                        // CAN handle
  uint32_t rx_fifo_num_;                           // CAN hardware fifo number
  osEventFlagsId_t can_rx_event_;                  // Rx CAN Interrupt Event
  osThreadId_t rx_task_handle_;                    // Rx Task Handle
  uint32_t rx_task_buffer_[ 256 ];                 // Rx Task Buffer
  StaticTask_t rx_task_control_block_;             // Rx Task Control Block
  const osThreadAttr_t rx_task_attributes_ =       // Rx Task Attributes
  {
    .name = "CAN Rx Handler",
    .cb_mem = &rx_task_control_block_,
    .cb_size = sizeof(rx_task_control_block_),
    .stack_mem = &rx_task_buffer_[0],
    .stack_size = sizeof(rx_task_buffer_),
    .priority = (osPriority_t) osPriorityAboveNormal1,
  };
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DRIVERS_INC_CAN_HPP_ */
