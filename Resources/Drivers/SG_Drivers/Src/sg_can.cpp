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

    // Start HAL CAN modules
    uint32_t error_code;

    error_code = HAL_CAN_Start(hcan1_);
    if (error_code != 0) { return error_code; }

    error_code = HAL_CAN_Start(hcan2_);
    if (error_code != 0) { return error_code; }

    HAL_CAN_ActivateNotification(hcan2_, CAN_IT_RX_FIFO0_MSG_PENDING);

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
    if (num_rx_msgs_ >= MAX_RX_MSGS)
        return -1;
    else
        rx_messages_.insert(etl::make_pair(msg->can_id, msg));
    
    // Update filter stuff
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE; /*Enable the filter*/
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   /*Mask mode*/
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;                  /*Accept everything*/
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  /*One 32-bit filter*/
    sFilterConfig.FilterBank = 0;                       /*Init bank 0*/
    sFilterConfig.FilterFIFOAssignment = 0;             /*Assign to FIFO 0*/
    HAL_CAN_ConfigFilter(hcan1_, &sFilterConfig);

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
        while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan2_));

        uint32_t txMailbox;
        CAN_TxHeaderTypeDef txHeader = {
            .StdId = tx_msg_->can_id,
            .ExtId = tx_msg_->can_id,
            .IDE = tx_msg_->id_type,
            .RTR = tx_msg_->rtr_mode,
            .DLC = tx_msg_->len,
        };

        HAL_CAN_AddTxMessage(hcan2_, &txHeader, tx_msg_->data, &txMailbox);
    }
}

void CANNode::Rx1Receive(void *argument) {
    while (1) {
        osEventFlagsWait(can_rx1_event_, 0x1, osFlagsWaitAny, osWaitForever);

        CAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        while (HAL_CAN_GetRxFifoFillLevel(hcan1_, CAN_RX_FIFO0)) {
            HAL_CAN_GetRxMessage(hcan1_, CAN_RX_FIFO0, &rxHeader, rxData);
            CANMessage* rx_msg = (*rx_messages_.find(rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId)).second;

            if (rx_msg != nullptr) {
                osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
                for (uint32_t i = 0; i < rx_msg->len; i++)
                    rx_msg->data[i] = rxData[i];
                osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
            }
            
            HAL_CAN_ActivateNotification(hcan1_, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
}

void CANNode::Rx2Receive(void *argument) {
    while (1) {
        osEventFlagsWait(can_rx2_event_, 0x1, osFlagsWaitAny, osWaitForever);

        CAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        while (HAL_CAN_GetRxFifoFillLevel(hcan2_, CAN_RX_FIFO0)) {
            HAL_CAN_GetRxMessage(hcan1_, CAN_RX_FIFO0, &rxHeader, rxData);
            CANMessage* rx_msg = (*rx_messages_.find(rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId)).second;

            if (rx_msg != nullptr) {
                osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
                for (uint32_t i = 0; i < rx_msg->len; i++)
                    rx_msg->data[i] = rxData[i];
                osMutexAcquire(rx_msg->mutex_id_, osWaitForever);
            }
            
            HAL_CAN_ActivateNotification(hcan2_, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Logger::LogInfo("CAN message received\n");
    CANNode::SetRxFlag(hcan);
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}