#include "sg_can.hpp"

CANNode::CANNode(CAN_HandleTypeDef* hcan1, CAN_HandleTypeDef* hcan2) {
    hcan1_ = hcan1;
    hcan2_ = hcan2;
}

int32_t CANNode::Start() {
    // Start tasks
    tx1_task_handle_ = osThreadNew((osThreadFunc_t)CANNode::Tx1Send, NULL, &tx1_task_attributes_);
    tx2_task_handle_ = osThreadNew((osThreadFunc_t)CANNode::Tx2Send, NULL, &tx2_task_attributes_);
    rx1_task_handle_ = osThreadNew((osThreadFunc_t)CANNode::Rx1Receive, NULL, &rx1_task_attributes_);
    rx2_task_handle_ = osThreadNew((osThreadFunc_t)CANNode::Rx2Receive, NULL, &rx2_task_attributes_);

    // Add all Rx messages to filters. Each filter bank can accept 2 IDs in 32-bit mode
    uint32_t n = 0;
    uint32_t filter_pair[2] = { 0x0, 0x0 };

    for (auto iter = rx_messages_.begin(); iter != rx_messages_.end(); ++iter) {
        if (iter->second->id_type == CAN_ID_STD) {
            filter_pair[n % 2] = iter->first << 21;
        }
        else {
            filter_pair[n % 2] = (iter->first << 3) & iter->second->id_type & iter->second->rtr_mode;
        }

        if (n % 2 == 1) {
            CAN_FilterTypeDef sFilterConfig = {
                .FilterIdHigh = filter_pair[0] >> 16,
                .FilterIdLow = filter_pair[0],
                .FilterMaskIdHigh = filter_pair[1] >> 16,
                .FilterMaskIdLow = filter_pair[1],
                .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                .FilterBank = n / 2,
                .FilterMode = CAN_FILTERMODE_IDLIST,
                .FilterScale = CAN_FILTERSCALE_32BIT,
                .FilterActivation = CAN_FILTER_ENABLE,
            };
            HAL_CAN_ConfigFilter(hcan1_, &sFilterConfig);
        }
        n++;
    }

    // Catch last ID if odd number of IDs
    if (n % 2 == 1) {
        CAN_FilterTypeDef sFilterConfig = {
            .FilterIdHigh = filter_pair[0] >> 16,
            .FilterIdLow = filter_pair[0],
            .FilterMaskIdHigh = filter_pair[1] >> 16,
            .FilterMaskIdLow = filter_pair[1],
            .FilterFIFOAssignment = CAN_FILTER_FIFO0,
            .FilterBank = n / 2,
            .FilterMode = CAN_FILTERMODE_IDLIST,
            .FilterScale = CAN_FILTERSCALE_32BIT,
            .FilterActivation = CAN_FILTER_ENABLE,
        };
        HAL_CAN_ConfigFilter(hcan1_, &sFilterConfig);
    }

    // Start HAL CAN modules
    uint32_t error_code;

    error_code = HAL_CAN_Start(hcan1_);
    if (error_code != 0) { return error_code; }

    //error_code = HAL_CAN_Start(hcan2_);
    if (error_code != 0) { return error_code; }

    HAL_CAN_ActivateNotification(hcan1_, CAN_IT_RX_FIFO0_MSG_PENDING);
    //HAL_CAN_ActivateNotification(hcan2_, CAN_IT_RX_FIFO0_MSG_PENDING);

    return 0;
}

int32_t CANNode::Send(CANMessage* msg) {
    tx_msg_ = msg;
    tx1_sent_ = false;
    tx2_sent_ = false;

    osEventFlagsSet(can_tx1_event_, 0x1);
    osEventFlagsSet(can_tx2_event_, 0x1);

    return 0;
}

int32_t CANNode::GetRxData(uint32_t can_id, uint8_t buf[]) {
    CANMessage* msg = (*rx_messages_.find(can_id)).second;
    if (msg != nullptr) {
        buf = msg->data;
        return 0;
    }
    else
        return -1;
}

int32_t CANNode::AddRxMessage(CANMessage* msg) {
    if (num_rx_msgs_ == MAX_RX_MSGS)
        return -1;

    rx_messages_.insert(etl::make_pair(msg->can_id, msg));

    num_rx_msgs_++;
    return 0;
}

int32_t CANNode::AddRxMessages(CANMessage* msg[], uint32_t len) {
    uint32_t error_code = 0;

    for (uint32_t i = 0; i < len; i++) {
        error_code = CANNode::AddRxMessage(msg[i]);
        if (error_code != 0) { return error_code; }
    }

    return 0;
}

int32_t CANNode::SetRxFlag(CAN_HandleTypeDef* hcan) {
    HAL_CAN_DeactivateNotification(hcan1_, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (hcan == hcan1_)
        return osEventFlagsSet(can_rx1_event_, 0x1);
    else if (hcan == hcan2_)
        return osEventFlagsSet(can_rx2_event_, 0x1);
    else    // Invalid CAN module
        return -1;
}

void CANNode::Tx1Send(void *argument) {
    while (1) {
        Logger::LogInfo("Sending...\n");
        osEventFlagsWait(can_tx1_event_, 0x1, osFlagsWaitAny, osWaitForever);

        // Spinlock until a tx mailbox is empty
        while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan1_));

        uint32_t txMailbox;
        CAN_TxHeaderTypeDef txHeader = {
            .StdId = tx_msg_->can_id,
            .ExtId = tx_msg_->can_id,
            .IDE = tx_msg_->id_type,
            .RTR = tx_msg_->rtr_mode,
            .DLC = tx_msg_->len,
        };

        HAL_CAN_AddTxMessage(hcan1_, &txHeader, tx_msg_->data, &txMailbox);
    }
}

void CANNode::Tx2Send(void *argument) {
    while (1) {
        Logger::LogInfo("Sending...\n");
        osEventFlagsWait(can_tx2_event_, 0x1, osFlagsWaitAny, osWaitForever);

        // Spinlock until a tx mailbox is empty
        // while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan2_));

        // uint32_t txMailbox;
        // CAN_TxHeaderTypeDef txHeader = {
        //     .StdId = tx_msg_->can_id,
        //     .ExtId = tx_msg_->can_id,
        //     .IDE = tx_msg_->id_type,
        //     .RTR = tx_msg_->rtr_mode,
        //     .DLC = tx_msg_->len,
        // };

        // HAL_CAN_AddTxMessage(hcan2_, &txHeader, tx_msg_->data, &txMailbox);
    }
}

void CANNode::Rx1Receive(void *argument) {
    while (1) {
        osEventFlagsWait(can_rx1_event_, 0x1, osFlagsWaitAny, osWaitForever);

        CAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        while(HAL_CAN_GetRxFifoFillLevel(hcan1_, CAN_RX_FIFO0)) {
            if (HAL_CAN_GetRxMessage(hcan1_, CAN_RX_FIFO0, &rxHeader, &rxData[0]) != HAL_OK) {
                Error_Handler();
            }
        }

        CANMessage* rx_msg = rx_messages_[rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId];

        if (rx_msg != nullptr) {
            osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
            for (uint32_t i = 0; i < rx_msg->len; i++)
                rx_msg->data[i] = rxData[i];
            osMutexRelease(rx_msg->mutex_id_);
        }

        HAL_CAN_ActivateNotification(hcan1_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

void CANNode::Rx2Receive(void *argument) {
    while (1) {
        osEventFlagsWait(can_rx2_event_, 0x1, osFlagsWaitAny, osWaitForever);

        // CAN_RxHeaderTypeDef rxHeader;
        // uint8_t rxData[8];

        // while (HAL_CAN_GetRxFifoFillLevel(hcan2_, CAN_RX_FIFO0)) {
        //     HAL_CAN_GetRxMessage(hcan1_, CAN_RX_FIFO0, &rxHeader, rxData);
        //     CANMessage* rx_msg = (*rx_messages_.find(rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId)).second;

        //     if (rx_msg != nullptr) {
        //         osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
        //         for (uint32_t i = 0; i < rx_msg->len; i++)
        //             rx_msg->data[i] = rxData[i];
        //         osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
        //     }
            
        //     HAL_CAN_ActivateNotification(hcan2_, CAN_IT_RX_FIFO0_MSG_PENDING);
        // }
    }
}

