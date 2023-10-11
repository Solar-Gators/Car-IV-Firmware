/*
 * SteeringController.cpp
 *
 *  Created on: Oct 29, 2021
 *      Author: John Carr
 */

#include <SteeringController.hpp>

namespace SolarGators {
namespace DataModules {

// TODO: Currently left turn and right turn are able to be true at the same time

  SteeringController::SteeringController()
  {
    button_handle_ = osThreadNew((osThreadFunc_t)&SteeringController::ButtonHandler, this, &button_attributes_);
    button_event_ = osEventFlagsNew(NULL);
    if (button_event_ == NULL)
    {
        Error_Handler();
    }
    button_queue_ = osMessageQueueNew(16, sizeof(uint16_t), &button_queue_attributes_);
    if (button_queue_ == NULL)
    {
        Error_Handler();
    }
  }

  SteeringController::~SteeringController() {
    osEventFlagsDelete(button_event_);
    osMessageQueueDelete(button_queue_);
  }

  void SteeringController::EnableLeftTurnSignal()
  {
    if(!hazards_)
      left_turn_ = true;
  }

  void SteeringController::DisableLeftTurnSignal()
  {
    left_turn_ = false;
  }

  void SteeringController::ToggleLeftTurnSignal()
  {
    if(left_turn_) {
      DisableLeftTurnSignal();
    } else {
      EnableLeftTurnSignal();
      DisableRightTurnSignal();
      hazards_ = false;
    }
  }

  void SteeringController::EnableRightTurnSignal()
  {
    if(!hazards_)
      right_turn_ = true;
  }

  void SteeringController::DisableRightTurnSignal()
  {
    right_turn_ = false;
  }

  void SteeringController::ToggleRightTurnSignal()
  {
    if(right_turn_) {
      DisableRightTurnSignal();
    } else {
      EnableRightTurnSignal();
      DisableLeftTurnSignal();
	  hazards_ = false;
    }
  }

  void SteeringController::EnableHazards()
  {
    DisableLeftTurnSignal();
    DisableRightTurnSignal();
    hazards_ = true;
  }

  void SteeringController::DisableHazards()
  {
    hazards_ = false;
  }

  void SteeringController::ToggleHazards()
  {
    if(hazards_) {
      DisableHazards();
  	} else {
      EnableHazards();
      DisableRightTurnSignal();
      DisableLeftTurnSignal();
    }
  }

  void SteeringController::SetBpsFault(bool fault)
  {
    bps_fault_ = fault;
  }

  void SteeringController::EnableCruise()
  {
    // Don't allow cruise to be enabled in reverse
    if(!reverse_)
      cruise_enable_ = true;
  }

  void SteeringController::DisableCruise()
  {
    cruise_enable_ = false;
  }
  void SteeringController::ToggleCruise()
  {
    if(cruise_enable_)
      DisableCruise();
    else
      EnableCruise();
  }

  void SteeringController::EnableHeadlights()
  {
    headlights_ = true;
  }

  void SteeringController::DisableHeadlights()
  {
    headlights_ = false;
  }

  void SteeringController::ToggleHeadlights()
  {
    if(headlights_)
      DisableHeadlights();
    else
      EnableHeadlights();
  }

  void SteeringController::EnableHorn()
  {
    horn_ = true;
  }

  void SteeringController::DisableHorn()
  {
    horn_ = false;
  }

  void SteeringController::ToggleHorn()
  {
    if(horn_)
      DisableHorn();
    else
      EnableHorn();
  }

  void SteeringController::IncreaseCruiseSpeed()
  {
	  if (!regen_) {
		  SetCruiseSpeed(cruise_speed_++);
	  } else {
		  IncreaseRegen();
	  }
  }

  void SteeringController::DecreaseCruiseSpeed()
  {
	  if (!regen_) {
		  SetCruiseSpeed(cruise_speed_--);
	  } else {
		  DecreaseRegen();
	  }
  }

  void SteeringController::SetCruiseSpeed(uint8_t speed)
  {
    // Make sure the the requested cruise speed is acceptable
    if(speed < Max_Cruise_Speed_ && speed > Min_Cruise_Speed_)
      cruise_speed_ = speed;
  }

  void SteeringController::IncreaseRegen() {
	  if (regen_ > 0 && regen_ < 3) {
		  regen_++;
	  }
  }

  void SteeringController::DecreaseRegen() {
	  if (regen_ > 1) {
		  regen_--;
	  }
  }

  void SteeringController::EnableRegen() {
	  if (!regen_) {
		  regen_ = 3;
	  }
  }

  void SteeringController::DisableRegen() {
	  regen_ = 0;
  }

  void SteeringController::EnableReverse()
  {
    // Disable cruise if switching into reverse (should never need to happen but just in case)
    DisableCruise();
    reverse_ = true;
  }

  void SteeringController::DisableReverse()
  {
    reverse_ = false;
  }

  void SteeringController::ToggleReverse()
  {
    if(reverse_)
      DisableReverse();
    else
      EnableReverse();
  }

  void SteeringController::EnableEco()
  {
    eco_enable_ = true;
  }

  void SteeringController::DisableEco()
  {
    eco_enable_ = false;
  }
  void SteeringController::ToggleEco()
  {
    if(eco_enable_)
      DisableEco();
    else
      EnableEco();
  }

  void SteeringController::HandlePress(uint16_t pin)
  {
    // Handles calling the action debouncing
    osEventFlagsSet(button_event_, 0x1);
    osMessageQueuePut(button_queue_, &pin, 0, 0);
  }

  void SteeringController::AddButton(Drivers::Button* button)
  {
    buttons_.insert(etl::make_pair(button->pin_, *button));
  }

  void SteeringController::ButtonHandler()
  {
    while(1)
    {
      // Wait for event to be set, no timeout
      osEventFlagsWait(button_event_, 0x1, osFlagsWaitAny, osWaitForever);
      while(osMessageQueueGetCount(button_queue_) > 0)
      {
        uint16_t pin;
        osMessageQueueGet(button_queue_, &pin, NULL, 0);
        Drivers::Button* button = &(*buttons_.find(pin)).second;
        if(button != nullptr)
          button->HandlePress();
      }
    }

  }

} /* namespace DataModules */
} /* namespace SolarGators */
