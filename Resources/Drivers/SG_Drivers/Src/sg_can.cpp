#include "sg_can.hpp"

/* Only STM32L4 supported for now */
#ifndef STM32L496xx
#error This driver is only compatible with STM32L496xx devices
#endif

/* Must be running FreeRTOS */
#if (USE_FREERTOS != 1)
#error This driver requires FreeRTOS
#endif


/**
  * @brief  Starts threads, starts CAN modules, enables interrupts
  * @retval HAL status
  */
#pragma GCC diagnostic ignored "-Wpmf-conversions"
HAL_StatusTypeDef CANDevice::Start() {
    tx_task_id_ = osThreadNew((osThreadFunc_t)&CANDevice::HandleTx, this, &tx_task_attributes_);
    rx_task_id_ = osThreadNew((osThreadFunc_t)&CANDevice::HandleRx, this, &rx_task_attributes_);

    if (HAL_CAN_Start(hcan_) != HAL_OK)
        return HAL_ERROR;

    // Enable interrupt
    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief  Sends CAN message
  * @retval HAL status
  */
HAL_StatusTypeDef CANDevice::Send(CANFrame *msg) {
    // Put message in queue, blocking if queue is full
    // Aliasing is not an issue because data is copied to queue (how big is queue performance penalty?)
    if (osMessageQueuePut(tx_queue_, &msg, 0, TX_TIMEOUT) != osOK) {
        HandleTxTimeout();
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Returns CAN device status
  * @retval CAN device status
  */
uint32_t CANDevice::GetStatus() {
    return HAL_OK;
}

/**
  * @brief  Sets Rx flag to activate Rx thread. Call from CANDeviceManager SetRxFlag().
  */
void CANDevice::SetRxFlag() {
    osEventFlagsSet(rx_event_flag_, 0x1);
}

/**
  * @brief  Tx thread. Sends message when Tx flag is set.
  * @param  manager Pointer to CANDeviceManager object
  */
void CANDevice::HandleTx(void* argument) {
    CANFrame* tx_msg;
    while (1) {
        // Block thread execution until message appears in queue
        osMessageQueueGet(tx_queue_, &tx_msg, NULL, osWaitForever);

        #ifdef USE_LOGGING
        Logger::LogDebug("Tx on device %x", hcan_->Instance);
        #endif

        // Get message header from CANFrame object
        osMutexAcquire(tx_msg->mutex_id_, osWaitForever);
        CAN_TxHeaderTypeDef txHeader = {
            .StdId = tx_msg->can_id,
            .ExtId = tx_msg->can_id,
            .IDE = tx_msg->id_type,
            .RTR = tx_msg->rtr_mode,
            .DLC = tx_msg->len,
            .TransmitGlobalTime = DISABLE,
        };
        osMutexRelease(tx_msg->mutex_id_);

        // Spinlock until a tx mailbox is empty
        while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan_));

        // txMailbox stores the mailbox that the message was sent from
        uint32_t txMailbox;
        // Request HAL message send
        HAL_CAN_AddTxMessage(hcan_, &txHeader, tx_msg->data, &txMailbox);
    }
}

/**
  * @brief  Rx thread. Handles received message when Rx flag is set.
  * @param  manager Pointer to CANDeviceManager object
  */
void CANDevice::HandleRx(void* argument) {
    CANFrame* rx_msg;
    while (1) {
        // Block thread execution until Rx flag is set from ISR
        osEventFlagsWait(rx_event_flag_, 0x1, osFlagsWaitAny, osWaitForever);      

        #ifdef USE_LOGGING
        Logger::LogDebug("Rx on device %x", hcan_->Instance);
        #endif
        
        CAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        // Retrieve all messages from FIFO
        while(HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO0)) {
            if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rxHeader, &rxData[0]) != HAL_OK) {
                Error_Handler();
            }

            #ifdef USE_LOGGING
            Logger::LogDebug("Rx ID: %x", rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId);
            Logger::LogDebug("RX Data: %x %x %x %x %x %x %x %x", rxData[7], rxData[6], rxData[5], rxData[4], rxData[3], rxData[2], rxData[1], rxData[0]);
            #endif

            // Find message in map
            // If it exists, set rx_msg to point to the CANFrame object
            auto messages_it = rx_messages_->find(rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId);
            rx_msg = messages_it == rx_messages_->end() ? nullptr : (*messages_it).second;

            if (rx_msg != nullptr) {
                osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
                memcpy(rx_msg->data, rxData, rx_msg->len);
                for (uint32_t i = 0; i < rx_msg->len; i++)
                    rx_msg->data[i] = rxData[i];
                osMutexRelease(rx_msg->mutex_id_);

                // Call Rx callback
                if (rx_msg->rxCallback)
                    rx_msg->rxCallback(rxData);
            }
        }

        // Re-enable Rx interrupt
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

/**
  * @brief  Tx timeout callback. Called when Tx/Rx timeout timer expires.
  * @param  task_id Pointer to timed out thread.
  */
#pragma GCC diagnostic ignored "-Wpmf-conversions"
void CANDevice::HandleTxTimeout() {
    #ifdef USE_LOGGING
    Logger::LogError("Tx Timeout on device %x", hcan_->Instance);
    #endif

    // Terminate thread
    osThreadTerminate(tx_task_id_);

    // Reset HAL CAN module
    HAL_CAN_DeInit(hcan_);
    HAL_CAN_MspDeInit(hcan_);
    HAL_CAN_MspInit(hcan_);
    HAL_CAN_Init(hcan_);

    // Restart thread
    tx_task_id_ = osThreadNew((osThreadFunc_t)&CANDevice::HandleTx, this, &tx_task_attributes_);

    return;
}

/**
  * @brief  Add CAN Device to CANController
  * @param  device Pointer to CANDevice object
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::AddDevice(CANDevice *device) {
    if (devices_.full())
        return HAL_ERROR;
    devices_.push_back(device);

    // CANDevice needs pointer to rx_messages_
    device->rx_messages_ = &rx_messages_;

    return HAL_OK;
}

/**
  * @brief  Add CAN message to CANController
  * @param  msg Pointer to CANFrame object
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::AddRxMessage(CANFrame *msg) {
    if (num_msgs_ >= MAX_RX_MSGS)
        return HAL_ERROR;

    // Add message to map
    rx_messages_.insert(etl::make_pair(msg->can_id, msg));
    num_msgs_++;

    return HAL_OK;
}

HAL_StatusTypeDef CANController::AddRxMessage(CANFrame *msg, void (*rxCallback)(uint8_t*)) {
    if (num_msgs_ >= MAX_RX_MSGS)
        return HAL_ERROR;

    msg->rxCallback = rxCallback;

    // Add message to map
    rx_messages_.insert(etl::make_pair(msg->can_id, msg));
    num_msgs_++;

    return HAL_OK;
}

/**
  * @brief  Add CAN messages to CANController
  * @param  msg Array of pointers to CANFrame objects
  * @param  num_msgs Number of CANFrame objects in array
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::AddRxMessages(CANFrame *msg[], uint32_t num_msgs) {
    if (num_msgs_ + num_msgs >= MAX_RX_MSGS)
        return HAL_ERROR;

    // Add messages to map
    for (uint32_t i = 0; i < num_msgs; i++) {
        rx_messages_.insert(etl::make_pair(msg[i]->can_id, msg[i]));
    }
    num_msgs_ += num_msgs;

    return HAL_OK;
}

/**
  * @brief  Set CANController to accept all messages as high priority
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::AddFilterAll() {
    // Mask of 0 ignores all bits
    CAN_FilterTypeDef sFilterConfig = {
        .FilterIdHigh = 0,
        .FilterIdLow = 0,
        .FilterMaskIdHigh = 0,
        .FilterMaskIdLow = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterActivation = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank = 0,
    };

    return HAL_CAN_ConfigFilter(devices_[0]->hcan_, &sFilterConfig);
}

/**
  * @brief  Set CANController to accept messages with specified CAN ID.
  * @param  can_id CAN ID to accept
  * @param  id_type CAN ID type (CAN_ID_STD or CAN_ID_EXT)
  * @param  rtr_mode CAN RTR mode (CAN_RTR_DATA or CAN_RTR_REMOTE)
  * @param  priority FIFO priority (CAN_PRIORITY_NORMAL or CAN_PRIORITY_HIGH)
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::AddFilterId(uint32_t can_id, uint32_t id_type, uint32_t rtr_mode, uint32_t priority) {
    // If all filters are used and last filter has mask (second ID) value occupied
    if (filters_.full() && filters_.front().FilterMaskIdLow != 0)
        return HAL_ERROR;

    // Define filter contents according to figure 523 in RM0351. Using 32-bit filters only
    uint32_t filter_id;
    if (id_type == CAN_ID_STD) {
        filter_id = (can_id << 21) | id_type | rtr_mode;
    }
    else if (id_type == CAN_ID_EXT) {
        filter_id = (can_id << 8) | id_type | rtr_mode;
    }
    else
        return HAL_ERROR;

    // If filter mask field of last filter is empty, populate that section of the filter
    if (filters_.size() > 0 && filters_.front().FilterMaskIdLow == 0) {
        filters_.front().FilterMaskIdHigh = (filter_id >> 16) & 0xFFFF;
        filters_.front().FilterMaskIdLow = filter_id & 0xFFFF;
        // If priority is already high, keep it high. If it is low, set it to the priority of the current ID
        if (filters_.front().FilterFIFOAssignment == CAN_RX_FIFO1)
            filters_.front().FilterFIFOAssignment = (priority == CAN_PRIORITY_HIGH) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    }
    // Else, create a new filter
    else {
        CAN_FilterTypeDef sFilterConfig = {
            .FilterIdHigh = (filter_id >> 16) & 0xFFFF,
            .FilterIdLow = filter_id & 0xFFFF,
            .FilterMaskIdHigh = 0,
            .FilterMaskIdLow = 0,
            .FilterFIFOAssignment = (priority == CAN_PRIORITY_HIGH) ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
            .FilterBank = filters_.size(),
            .FilterMode = CAN_FILTERMODE_IDLIST,
            .FilterScale = CAN_FILTERSCALE_32BIT,
            .FilterActivation = CAN_FILTER_ENABLE,
            .SlaveStartFilterBank = 0,
        };
        filters_.push_front(sFilterConfig);
    }    
    
    return HAL_OK;
}

/**
  * @brief  Set CANController to accept messages with specified CAN ID and mask.
  * @param  can_id CAN ID to accept
  * @param  id_type CAN ID type (CAN_ID_STD or CAN_ID_EXT)
  * @param  rtr_mode CAN RTR mode (CAN_RTR_DATA or CAN_RTR_REMOTE)
  * @param  priority FIFO priority (CAN_PRIORITY_NORMAL or CAN_PRIORITY_HIGH)
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::AddFilterIdRange(uint32_t can_id, uint32_t range, uint32_t id_type, uint32_t rtr_mode, uint32_t priority) {
    if (filters_.full())
        return HAL_ERROR;
    if (range == 0)
        return HAL_ERROR;

    // Find position of rightmost 1 in range
    int32_t n = -1;
    uint32_t counter = range;
    while (counter) {
        counter >>= 1;
        n++;
    }
    // If range is not a power of 2, go to the next position to capture the entire range
    if (range > (uint32_t)(1 << n))
        n++;
    // Set id mask to be all ones except for the n least significant bits
    uint32_t id_mask = (0xFFFFFFFF >> n) << n;
    

    // Define filter contents according to figure 523 in RM0351. Using 32-bit filters only
    uint32_t filter_id;
    uint32_t filter_mask;
    if (id_type == CAN_ID_STD) {
        filter_id = (can_id << 21) | id_type | rtr_mode;
        filter_mask = id_mask << 21 | 0b110;
    }
    else if (id_type == CAN_ID_EXT) {
        filter_id = (can_id << 8) | id_type | rtr_mode;
        filter_mask = id_mask << 8 | 0b110;
    }
    else
        return HAL_ERROR;

    CAN_FilterTypeDef sFilterConfig = {
        .FilterIdHigh = (filter_id >> 16) & 0xFFFF,
        .FilterIdLow = filter_id & 0xFFFF,
        .FilterMaskIdHigh = (filter_mask >> 16) & 0xFFFF,
        .FilterMaskIdLow = filter_mask & 0xFFFF,
        .FilterFIFOAssignment = (priority == CAN_PRIORITY_HIGH) ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
        .FilterBank = filters_.size(),
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterActivation = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank = 0,
    };

    filters_.push_back(sFilterConfig);
    
    return HAL_OK;
}

/**
  * @brief  Start all CAN devices, configure filters
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::Start() {
    // Add filters to each device
    for (auto filter : filters_)
        for (auto device : devices_)
            HAL_CAN_ConfigFilter(device->hcan_, &filter);

    // Start all devices
    for (auto device : devices_)
        if (device->Start() != HAL_OK)
            return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief  Send CAN message on all CAN devices
  * @param  msg Pointer to CANFrame object
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::Send(CANFrame *msg) {
    for (auto device : devices_)
        if (device->Send(msg) != HAL_OK)
            return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief  Send CAN message on CAN device
  * @param  device Pointer to CANDevice object
  * @param  msg Pointer to CANFrame object
  * @retval HAL status
  */
HAL_StatusTypeDef CANController::SendOnDevice(CANDevice *device, CANFrame *msg) {
    return device->Send(msg);
}

/**
  * @brief  Get CAN message from CANController
  * @param  can_id CAN ID of message to get
  * @param  msg Pointer to CANFrame object
  * @retval HAL status
  */
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
HAL_StatusTypeDef CANController::GetMessage(uint32_t can_id, CANFrame *msg) {
    if (rx_messages_.find(can_id) == rx_messages_.end())
        return HAL_ERROR;

    msg = rx_messages_[can_id];

    return HAL_OK;
}

/**
  * @brief  Get CAN device status
  * @param  device Pointer to CANDevice object
  * @retval CAN device status
  */
HAL_StatusTypeDef CANController::GetDeviceStatus(CANDevice *device) {
    device->GetStatus();

    return HAL_OK;
}

/**
  * @brief  CAN Rx interrupt callback. Calls CANDeviceManager SetRxFlag().
  * @param  hcan Pointer to CAN_HandleTypeDef object
  */
void CANController::RxCallback(CAN_HandleTypeDef *hcan) {
    for (auto device : devices_)
        if (device->hcan_ == hcan) {
            device->SetRxFlag();
            break;
        }
}

/**
 * @brief  CAN Rx interrupt callback. Calls CANDeviceManager SetRxFlag().
 * @param  hcan Pointer to CAN_HandleTypeDef object
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    CANController::RxCallback(hcan);
}