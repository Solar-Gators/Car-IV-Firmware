/*
 * CANMessage.hpp
 *
 *  Created on: July 10, 2023
 *      Author: Matthew Shen
 */

#ifndef SG_CAN_HPP_
#define SG_CAN_HPP_

#include "main.h"
#include "stdint.h"
#include "etl/map.h"
#include "cmsis_os.h"

#include "logger.hpp"

#define MAX_RX_MSGS 64                      /* Size of Rx message map */

class CANMessage {
public:
    CANMessage(uint32_t can_id, uint32_t id_type, uint32_t len):
        can_id(can_id), id_type(id_type), len(len) {
        mutex_id_ = osMutexNew(&mutex_attributes_);
    };
    CANMessage(uint32_t can_id, 
                uint32_t id_type, 
                uint32_t len,
                void (*rxCallback)(uint8_t data[])):
        can_id(can_id), id_type(id_type), len(len), rxCallback(rxCallback) {
        mutex_id_ = osMutexNew(&mutex_attributes_);
    };

    uint32_t            can_id;             /* CAN ID, can be standard or extended */
    uint32_t            id_type;            /* CAN ID type, 0 if standard ID, 4 if extended ID */
    uint32_t            rtr_mode;           /* RTR mode, 0 if not RTR message, 2 if RTR */
    uint32_t            len;                /* payload data length */
    uint8_t             data[8];            /* payload data array, maximum of 8 bytes */
    void (*rxCallback)(uint8_t data[]);     /* pointer to rx callback function */

    osMutexId_t         mutex_id_;          /* mutex id for message */
protected:
    StaticSemaphore_t   mutex_control_block_;
    const osMutexAttr_t mutex_attributes_ = {
        .name = "DMM",
        .attr_bits = osMutexRecursive,
        .cb_mem = &mutex_control_block_,
        .cb_size = sizeof(mutex_control_block_),
    };
};

class CANNode {
public:
    CANNode(CAN_HandleTypeDef* hcan1, CAN_HandleTypeDef* hcan2);    /* Constructor requires 2 HAL CAN instances */
    CANNode(CANNode &other) = delete;                               /* Copying is banned */
    void operator=(const CANNode &) = delete;                       /* = assignment is banned */

    static int32_t Start();                                         /* Starts all tasks, calls HAL_CAN_Start on all interfaces */

    static int32_t Send(CANMessage* msg);              /* Requests message send on all interfaces */

    static int32_t GetRxData(uint32_t can_id, uint8_t buf[]);       /* Retrieve data from received message with specified ID */

    static int32_t AddRxMessage(CANMessage* msg);      /* Adds message to receive list. Updates filter config to include message */

    static int32_t AddRxMessages(CANMessage* msg[], uint32_t len);     /* Adds messages to receive list. Updates filter config to include messages */

    static int32_t SetRxFlag(CAN_HandleTypeDef* hcan);              /* Notifies RX handler task that message has been received. Should be called from IRQ */

protected:

    /* Tx1 task definitions */
    static inline osEventFlagsId_t can_tx1_event_ = osEventFlagsNew(NULL);
    static inline osThreadId_t tx1_task_handle_;
    static inline uint32_t tx1_task_buffer_[1024];
    static inline StaticTask_t tx1_task_control_block_;
    constexpr static const osThreadAttr_t tx1_task_attributes_ = {
        .name = "CAN Tx1 Handler",
        .cb_mem = &tx1_task_control_block_,
        .cb_size = sizeof(tx1_task_control_block_),
        .stack_mem = &tx1_task_buffer_[0],
        .stack_size = sizeof(tx1_task_buffer_),
        .priority = (osPriority_t) osPriorityRealtime,
    };

    /* Tx2 task definitions */
    static inline osEventFlagsId_t can_tx2_event_ = osEventFlagsNew(NULL);
    static inline osThreadId_t tx2_task_handle_;
    static inline uint32_t tx2_task_buffer_[1024];
    static inline StaticTask_t tx2_task_control_block_;
    constexpr static const osThreadAttr_t tx2_task_attributes_ = {
        .name = "CAN Tx2 Handler",
        .cb_mem = &tx2_task_control_block_,
        .cb_size = sizeof(tx2_task_control_block_),
        .stack_mem = &tx2_task_buffer_[0],
        .stack_size = sizeof(tx2_task_buffer_),
        .priority = (osPriority_t) osPriorityRealtime,
    };

    /* Rx1 task definitions */
    static inline osEventFlagsId_t can_rx1_event_ = osEventFlagsNew(NULL);
    static inline osThreadId_t rx1_task_handle_;
    static inline uint32_t rx1_task_buffer_[1024];
    static inline StaticTask_t rx1_task_control_block_;
    constexpr static const osThreadAttr_t rx1_task_attributes_ = {
        .name = "CAN Rx1 Handler",
        .cb_mem = &rx1_task_control_block_,
        .cb_size = sizeof(rx1_task_control_block_),
        .stack_mem = &rx1_task_buffer_[0],
        .stack_size = sizeof(rx1_task_buffer_),
        .priority = (osPriority_t) osPriorityRealtime,
    };

    /* Rx2 task definitions */
    static inline osEventFlagsId_t can_rx2_event_ = osEventFlagsNew(NULL);
    static inline osThreadId_t rx2_task_handle_;
    static inline uint32_t rx2_task_buffer_[1024];
    static inline StaticTask_t rx2_task_control_block_;
    constexpr static const osThreadAttr_t rx2_task_attributes_ = {
        .name = "CAN Rx2 Handler",
        .cb_mem = &rx2_task_control_block_,
        .cb_size = sizeof(rx2_task_control_block_),
        .stack_mem = &rx2_task_buffer_[0],
        .stack_size = sizeof(rx2_task_buffer_),
        .priority = (osPriority_t) osPriorityRealtime,
    };

    
    static inline CAN_HandleTypeDef* hcan1_;            /* Pointer to HAL CAN module */
    static inline CAN_HandleTypeDef* hcan2_;            /* Pointer to HAL CAN module */

    static inline CANMessage* tx_msg_;     /* Current message to send */
    static inline etl::atomic<bool> tx1_sent_ = true;   /* True if tx_msg_ was successfully sent on hcan1_, false if failed to send */
    static inline etl::atomic<bool> tx2_sent_ = true;   /* True if tx_msgs_ was successfully sent on hcan2_, false if failed to send */
    
    static inline uint32_t num_rx_msgs_ = 0;                                                /* Number of stored Rx messages */
    static inline etl::map<uint32_t, CANMessage*, MAX_RX_MSGS> rx_messages_;   /* Map to store all receivable messages */
    static inline etl::atomic<bool> rx_func_executed_ = false;


    static void Tx1Send(void *argument);     /* Receive hcan2_ message, update rx_messages_, call message funcptr */
    static void Tx2Send(void *argument);        /* Send message on hcan2_ */
    static void Rx1Receive(void *argument);     /* Receive hcan1_ message, update rx_messages_, call message funcptr */
    static void Rx2Receive(void *argument);     /* Receive hcan2_ message, update rx_messages_, call message funcptr */
};

#endif  /* SG_CAN_HPP_ */