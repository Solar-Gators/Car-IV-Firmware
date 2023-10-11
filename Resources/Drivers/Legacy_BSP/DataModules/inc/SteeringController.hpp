/*
 * SteeringController.hpp
 *
 *  Created on: Oct 29, 2021
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DATAMODULES_INC_STEERINGCONTROLLER_HPP_
#define SOLARGATORSBSP_DATAMODULES_INC_STEERINGCONTROLLER_HPP_

#include <functional>
#include "cmsis_os2.h"

#include "etl/map.h"

#include "Steering.hpp"
#include "Button.hpp"
#include "LED.hpp"

namespace SolarGators {
namespace DataModules {

class SteeringController final : public Steering {
public:
  // Constructor / Destructor
  SteeringController();
  virtual ~SteeringController();
  // Setters
  // Left Turn
  void EnableLeftTurnSignal();
  void DisableLeftTurnSignal();
  void ToggleLeftTurnSignal();
  void HandleLeftTurnSignal();
  // Right Turn
  void EnableRightTurnSignal();
  void DisableRightTurnSignal();
  void ToggleRightTurnSignal();
  void HandleRightTurnSignal();
  // Hazards
  void EnableHazards();
  void DisableHazards();
  void ToggleHazards();
  // Bps Fault
  void SetBpsFault(bool fault);
  // Cruise
  void EnableCruise();
  void DisableCruise();
  void ToggleCruise();
  // Headlights
  void EnableHeadlights();
  void DisableHeadlights();
  void ToggleHeadlights();
  // Horn
  void EnableHorn();
  void DisableHorn();
  void ToggleHorn();
  // Cruise Speed
  void IncreaseCruiseSpeed();
  void DecreaseCruiseSpeed();
  void SetCruiseSpeed(uint8_t speed);
  // Regen
  void IncreaseRegen();
  void DecreaseRegen();
  void EnableRegen();
  void DisableRegen();
  // Reverse
  void EnableReverse();
  void DisableReverse();
  void ToggleReverse();
  // Eco
  void EnableEco();
  void DisableEco();
  void ToggleEco();
  // Setup Functions
  void AddButton(Drivers::Button* button);
  // Handle User Input
  void HandlePress(uint16_t pin);
private:
  // Turn Lights thread
  void TurnLightsUpdater();
  void ButtonHandler();
  // Button Stuff
  osEventFlagsId_t button_event_;                                         // Button Event
  osMessageQueueId_t button_queue_;                                       // Button Queue
  uint8_t button_queue_buffer_[ 16 * sizeof( uint16_t ) ];                // Button Queue Buffer
  StaticQueue_t  button_queue_control_block_;                             // Button Queue Control Block
  const osMessageQueueAttr_t button_queue_attributes_ =                   // Button Queue Attributes
  {
    .name = "Button Queue",
    .cb_mem = &button_queue_control_block_,
    .cb_size = sizeof(button_queue_control_block_),
    .mq_mem = &button_queue_buffer_,
    .mq_size = sizeof(button_queue_buffer_)
  };
  osThreadId_t button_handle_;                                            // Button Task Handle
  uint32_t button_buffer_[ 128 ];                                          // Button Task Buffer
  StaticTask_t button_control_block_;                                     // Button Task Control Block
  const osThreadAttr_t button_attributes_ =                               // Button Task Attributes
  {
    .name = "Button Handler",
    .cb_mem = &button_control_block_,
    .cb_size = sizeof(button_control_block_),
    .stack_mem = &button_buffer_[0],
    .stack_size = sizeof(button_buffer_),
    .priority = (osPriority_t) osPriorityRealtime7,
  };

  ::etl::map<uint16_t, Drivers::Button, 10> buttons_;
};

} /* namespace DataModules */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DATAMODULES_INC_STEERINGCONTROLLER_HPP_ */
