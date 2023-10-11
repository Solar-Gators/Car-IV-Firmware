/*
 * CAN.cpp
 *
 *  Created on: Oct 29, 2021
 *      Author: John Carr
 */

#include <CAN.hpp>

//using namespace SolarGators;

extern SolarGators::DataModules::DataModule* BMS_Rx_0_ptr;
extern SolarGators::DataModules::DataModule* BMS_Rx_1_ptr;
extern SolarGators::DataModules::DataModule* BMS_Rx_2_ptr;
extern SolarGators::DataModules::DataModule* BMS_Rx_4_ptr;
extern SolarGators::DataModules::DataModule* Motor_Rx_0_ptr;
extern SolarGators::DataModules::DataModule* Motor_Rx_2_ptr;
extern SolarGators::DataModules::DataModule* FLights_ptr;
extern SolarGators::DataModules::DataModule* RLights_ptr;
extern SolarGators::DataModules::DataModule* PowerBoard_ptr;
extern SolarGators::DataModules::DataModule* MPPT0_ptr;
extern SolarGators::DataModules::DataModule* MPPT1_ptr;
extern SolarGators::DataModules::DataModule* MPPT2_ptr;

namespace SolarGators {
namespace Drivers {

CANDriver::CANDriver(CAN_HandleTypeDef* hcan, uint32_t rx_fifo_num_):hcan_(hcan),rx_fifo_num_(rx_fifo_num_)
{

}

void CANDriver::Init()
{
  // Configure Filter
  //Initialize a hardware filter that passes everything
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE; /*Enable the filter*/
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   /*Mask mode*/
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;                  /*Accept everything*/
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  /*One 32-bit filter*/
  sFilterConfig.FilterBank = 0;                       /*Init bank 0*/
  sFilterConfig.FilterFIFOAssignment = 0;             /*Assign to FIFO 0*/
  HAL_CAN_ConfigFilter(hcan_, &sFilterConfig);

//  can_rx_event_ = osEventFlagsNew(NULL);
//  if (can_rx_event_ == NULL)
//  {
//      Error_Handler();
//  }

  rx_task_handle_ = osThreadNew((osThreadFunc_t)&CANDriver::HandleReceive, this, &rx_task_attributes_);
  if (rx_task_handle_ == NULL)
  {
      Error_Handler();
  }
  //HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan_);
}

CANDriver::~CANDriver()
{ }

void CANDriver::HandleReceive()
{
  while(1)
  {
    //osEventFlagsWait(can_rx_event_, 0x1, osFlagsWaitAny, osWaitForever);
	//osSemaphoreAcquire(canSem, osWaitForever);

    CAN_RxHeaderTypeDef pHeader;
    uint8_t aData[MAX_DATA_SIZE];

    while(HAL_CAN_GetRxFifoFillLevel(hcan_, rx_fifo_num_))
    {
      HAL_CAN_GetRxMessage(hcan_, rx_fifo_num_, &pHeader, aData);
      //DataModules::DataModule* rx_module = (*modules_.find(pHeader.IDE == CAN_ID_STD ? pHeader.StdId : pHeader.ExtId)).second;

//		  if (pHeader.StdId == FLights_ptr->can_id_) {
//
//			  osMutexAcquire(FLights_ptr->mutex_id_, osWaitForever);
//			  FLights_ptr->FromByteArray(aData);
//			  osMutexRelease(FLights_ptr->mutex_id_);
//			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
//
//		  } else
//      	  if (pHeader.StdId == BMS_Rx_0_ptr->can_id_) {
//
//			  osMutexAcquire(BMS_Rx_0_ptr->mutex_id_, osWaitForever);
//			  BMS_Rx_0_ptr->FromByteArray(aData);
//			  osMutexRelease(BMS_Rx_0_ptr->mutex_id_);
//			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
//
//		  } else if (pHeader.StdId == BMS_Rx_2_ptr->can_id_) {
//
//			  osMutexAcquire(BMS_Rx_2_ptr->mutex_id_, osWaitForever);
//			  BMS_Rx_2_ptr->FromByteArray(aData);
//			  osMutexRelease(BMS_Rx_2_ptr->mutex_id_);
//			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
//
//		  } else if (pHeader.StdId == BMS_Rx_4_ptr->can_id_) {
//
//			  osMutexAcquire(BMS_Rx_4_ptr->mutex_id_, osWaitForever);
//			  BMS_Rx_4_ptr->FromByteArray(aData);
//			  osMutexRelease(BMS_Rx_4_ptr->mutex_id_);
//			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
//
//		  } else if (pHeader.ExtId == Motor_Rx_0_ptr->can_id_) {
//
//			  osMutexAcquire(Motor_Rx_0_ptr->mutex_id_, osWaitForever);
//			  Motor_Rx_0_ptr->FromByteArray(aData);
//			  osMutexRelease(Motor_Rx_0_ptr->mutex_id_);
//			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
//
//			} else if (pHeader.ExtId == Motor_Rx_2_ptr->can_id_) {
//
//			  osMutexAcquire(Motor_Rx_2_ptr->mutex_id_, osWaitForever);
//			  Motor_Rx_2_ptr->FromByteArray(aData);
//			  osMutexRelease(Motor_Rx_2_ptr->mutex_id_);
//			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
//
//			}

      	  if (pHeader.IDE == CAN_ID_STD) {

      		  if (pHeader.StdId == BMS_Rx_0_ptr->can_id_) {

				  osMutexAcquire(BMS_Rx_0_ptr->mutex_id_, osWaitForever);
				  BMS_Rx_0_ptr->FromByteArray(aData);
				  osMutexRelease(BMS_Rx_0_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == BMS_Rx_1_ptr->can_id_) {

				  osMutexAcquire(BMS_Rx_1_ptr->mutex_id_, osWaitForever);
				  BMS_Rx_1_ptr->FromByteArray(aData);
				  osMutexRelease(BMS_Rx_1_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == BMS_Rx_2_ptr->can_id_) {

				  osMutexAcquire(BMS_Rx_2_ptr->mutex_id_, osWaitForever);
				  BMS_Rx_2_ptr->FromByteArray(aData);
				  osMutexRelease(BMS_Rx_2_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == BMS_Rx_4_ptr->can_id_) {

				  osMutexAcquire(BMS_Rx_4_ptr->mutex_id_, osWaitForever);
				  BMS_Rx_4_ptr->FromByteArray(aData);
				  osMutexRelease(BMS_Rx_4_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == PowerBoard_ptr->can_id_) {

				  osMutexAcquire(PowerBoard_ptr->mutex_id_, osWaitForever);
				  PowerBoard_ptr->FromByteArray(aData);
				  osMutexRelease(PowerBoard_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == FLights_ptr->can_id_) {

				  osMutexAcquire(FLights_ptr->mutex_id_, osWaitForever);
				  FLights_ptr->FromByteArray(aData);
				  osMutexRelease(FLights_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == RLights_ptr->can_id_) {

				  osMutexAcquire(RLights_ptr->mutex_id_, osWaitForever);
				  RLights_ptr->FromByteArray(aData);
				  osMutexRelease(RLights_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == MPPT0_ptr->can_id_) {

				  osMutexAcquire(MPPT0_ptr->mutex_id_, osWaitForever);
				  MPPT0_ptr->FromByteArray(aData);
				  osMutexRelease(MPPT0_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == MPPT1_ptr->can_id_) {

				  osMutexAcquire(MPPT1_ptr->mutex_id_, osWaitForever);
				  MPPT1_ptr->FromByteArray(aData);
				  osMutexRelease(MPPT1_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  } else if (pHeader.StdId == MPPT2_ptr->can_id_) {

				  osMutexAcquire(MPPT2_ptr->mutex_id_, osWaitForever);
				  MPPT2_ptr->FromByteArray(aData);
				  osMutexRelease(MPPT2_ptr->mutex_id_);
				  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			  }

      	  } else if (pHeader.IDE == CAN_ID_EXT) {

      		if (pHeader.ExtId == Motor_Rx_0_ptr->can_id_) {

			  osMutexAcquire(Motor_Rx_0_ptr->mutex_id_, osWaitForever);
			  Motor_Rx_0_ptr->FromByteArray(aData);
			  osMutexRelease(Motor_Rx_0_ptr->mutex_id_);
			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			} else if (pHeader.ExtId == Motor_Rx_2_ptr->can_id_) {

			  osMutexAcquire(Motor_Rx_2_ptr->mutex_id_, osWaitForever);
			  Motor_Rx_2_ptr->FromByteArray(aData);
			  osMutexRelease(Motor_Rx_2_ptr->mutex_id_);
			  HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);

			}

      	  }

//		  if(rx_module != nullptr)
//		  {
//			osMutexAcquire(rx_module->mutex_id_, osWaitForever);
			//HAL_GPIO_TogglePin(STM_OK_GPIO_Port, STM_OK_Pin);
			//rx_module->FromByteArray(aData);
//			osMutexRelease(rx_module->mutex_id_);
//		  }
    }
    //osDelay(2);
    //osSemaphoreRelease(canSem);
    //HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
    osDelay(10);
  }
}

void CANDriver::Send(SolarGators::DataModules::DataModule* data)
{
  //Spinlock until a tx mailbox is empty
  while(!HAL_CAN_GetTxMailboxesFreeLevel(hcan_));

  //Initialize Header
  uint32_t pTxMailbox;
  CAN_TxHeaderTypeDef pHeader;
  pHeader.RTR = data->is_rtr_ ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  pHeader.DLC = data->size_;
  if(data->is_ext_id_)
  {
    pHeader.ExtId = data->can_id_;
    pHeader.IDE = CAN_ID_EXT;
  }
  else
  {
    pHeader.StdId = data->can_id_;
    pHeader.IDE = CAN_ID_STD;
  }
  //Put CAN message in tx mailbox
  uint8_t aData[MAX_DATA_SIZE];
  osMutexAcquire(data->mutex_id_, osWaitForever);
  data->ToByteArray(aData);
  osMutexRelease(data->mutex_id_);
  HAL_CAN_AddTxMessage(hcan_, &pHeader, aData, &pTxMailbox);
}

bool CANDriver::AddRxModule(DataModules::DataModule* module)
{
  modules_.insert(etl::make_pair(module->can_id_, module));
  // TODO: Check if successful insertion
  return true;
}

bool CANDriver::RemoveRxModule(uint32_t module_id)
{
  // TODO: Implement (Remove needs to be added to tree container)
  return false;
}

void CANDriver::SetRxFlag()
{
  osEventFlagsSet(can_rx_event_, 0x1);
}

} /* namespace Drivers */
} /* namespace SolarGators */
