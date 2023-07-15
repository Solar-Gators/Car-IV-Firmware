/*
 * CANMessage.hpp
 *
 *  Created on: July 10, 2023
 *      Author: Matthew Shen
 */

#ifndef SG_CAN_HPP_
#define SG_CAN_HPP_

#define STM32L4xx       /* Development only, remove in final project */
#define CAN1            /* Development only, remove in final project */

#ifdef STM32L4xx
#include "stm32l4xx_hal_can.h"
#endif

#include <stdint.h>
#include "etl/map.h"
#include "cmsis_os.h"

namespace SolarGators {

typedef enum {
    SG_CAN_ID_STD = 0x0,           /* standard ID */
    SG_CAN_ID_EXT = 0x4            /* extended ID */
} sg_can_IdType_e;

class CANMessage {
public:
    uint32_t            can_id;                 /* CAN ID, can be standard or extended */
    sg_can_IdType_e     id_type;                /* CAN ID type */
    uint32_t            data_length;            /* payload data length */
    uint8_t             data[8];                /* payload data array, maximum of 8 bytes */
    void (*rxCallback)(uint8_t data[]);         /* pointer to rx callback function */
};

class CANNode {
public:
    CANNode() = default;
    CANNode(CAN_HandleTypeDef* hcan);
    CANNode(CANNode &other) = delete;               /* Copying is banned */
    void operator=(const CANNode &) = delete;       /* = assignment is banned */

    static int32_t Start();

    static int32_t Send(SolarGators::CANMessage* msg);

    static int32_t AddRxMessage(SolarGators::CANMessage* msg);

    static int32_t AddRxMessages(SolarGators::CANMessage* msg[], uint32_t len);

    static int32_t AddInterface(CAN_HandleTypeDef* hcan);

    static int32_t SetRxFlag(CAN_HandleTypeDef* hcan);


protected:
    // Tx1 Task
    static osEventFlagsId_t can_tx1_event_;
    static osThreadId_t tx1_task_handle_;
    static uint32_t tx1_task_buffer_[32];
    static StaticTask_t tx1_task_control_block_;
    static const osThreadAddr_t tx1_task_attributes_ = {
        .name = "CAN TX1 Handler",
        .cb_mem = &tx1_task_control_block_,
        .stack_mem = &tx1_task_buffer_[0],
        .stack_size = sizeof(tx1_task_buffer_),
        .priority = (osPriority_t) os PriorityRealtime,
    };

    // Rx1 Task
    static osEventFlagsId_t can_rx_event_;
    static osThreadId_t rx_task_handle_;
    static uint32_t rx_task_buffer_[128];
    static StaticTask_t rx_task_control_block_;
    static const osThreadAttr_t rx_task_attributes_ = {
        .name = "CAN RX Handler",
        .cb_mem = &rx_task_control_block_,
        .cb_size = sizeof(rx_task_control_block_),
        .stack_mem = &rx_task_buffer_[0],
        .stack_size = sizeof(rx_task_buffer_),
        .priority = (osPriority_t) osPriorityRealtime,
    };

    static SolarGators::CANMessage* tx_msg_;

    static uint32_t num_interfaces_;
    static CAN_HandleTypeDef* interfaces_[2];

    static uint32_t num_msgs_;
    static ::etl::map<uint32_t, SolarGators::CANMessage*, 64> rx_messages_;

    int32_t Tx1Send();
    int32_t Tx2Send();
    void HandleReceive();
};

}   /* namespace SolarGators */

#endif  /* SG_CAN_HPP_ */






uint32_t SolarGators::CANNode::num_interfaces_ = 0;
uint32_t SolarGators::CANNode::num_msgs_ = 0;
CAN_HandleTypeDef* SolarGators::CANNode::interfaces_[2] = {nullptr, nullptr};


SolarGators::CANNode::CANNode(CAN_HandleTypeDef* hcan) {
    interfaces_[0] = hcan;
    num_interfaces_ = 1;
}

int32_t SolarGators::CANNode::Start() {
    // Start can on all interfaces
    for (uint32_t i = 0; i < num_interfaces_; i++)
        HAL_CAN_Start(interfaces_[i]);


    tx1_task_handle_ = osThreadNew((osThreadFunc_t)&CANNode::Tx1Send, this, &tx1_task_attributes_);


    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

int32_t SolarGators::CANNode::Send(SolarGators::CANMessage* msg) {
    // Wait for one interface to become free

    CAN_TxHeaderTypeDef hal_can_header = {
        .StdId = msg->can_id,
        .ExtId = msg->can_id,
        .IDE = msg->id_type,
        .RTR = CAN_RTR_DATA,
        .DLC = msg->data_length,
    };

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(interfaces_[0], &hal_can_header, msg->data, &tx_mailbox);

    return 0;
}

int32_t SolarGators::CANNode::AddRxMessage(SolarGators::CANMessage* msg) {
    rx_messages_.insert(etl::make_pair(msg->can_id, msg));
    return 0;
}

int32_t SolarGators::CANNode::AddRxMessages(SolarGators::CANMessage* msg[], uint32_t len) {
    int32_t error_code = 0;
    for (uint32_t i = 0; i < len; i++)
        error_code += SolarGators::CANNode::AddRxMessage(msg[i]);
    return error_code;
}

int32_t SolarGators::CANNode::SetRxFlag(CAN_HandleTypeDef *hcan) {
    osEventFlagsSet(can_rx_event_, 0x1);
    return 0;
}

void SolarGators::CANNode::HandleReceive() {
    while (1) {
        // Wait for rx flag to be set in IRQ
        osEventFlagsWait(can_rx_event_, 0x1, osFlagsWaitAny, osWaitForever);
    }
}







// Put in user code
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    SolarGators::CANNode::SetRxFlag(hcan);
}