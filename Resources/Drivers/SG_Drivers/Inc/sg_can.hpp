/*
 * sg_can.hpp
 *
 *  Created on: July 10, 2023
 *      Author: Matthew Shen
 */

#ifndef SG_CAN_HPP_
#define SG_CAN_HPP_

#include "main.h"
#include "stdint.h"
#include "etl/map.h"
#include "etl/vector.h"
#include "etl/deque.h"
#include "cmsis_os2.h"

#include "logger.hpp"

#define THREAD_PRIORITY osPriorityAboveNormal   /* Priority of Rx and Tx threads */

#define TX_QUEUE_SIZE 3                         /* Size of Tx message queue */
#define TX_TIMEOUT    10                        /* Timeout for tx thread in ms */

#define NUM_FILTER_BANKS 14                     /* 14 filters on single CAN, 28 on dual CAN */
#define MAX_RX_MSGS      28                     /* Size of Rx message map */

#define CAN_PRIORITY_NORMAL 0                   /* Interrupt waits for FIFO full */
#define CAN_PRIORITY_HIGH   1                   /* Interrupt waits for FIFO pending */
#define CAN_FILTER_ALL      0                   /* Accept all messages */
#define CAN_FILTER_CUSTOM   2                   /* Accept messages in rx_messages_ */

/**
 * Class for storing information needed to transmit a single CAN frame.
 * 
 * @param can_id        CAN ID, stores standard or extended ID types
 * @param id_type       CAN ID type, can be CAN_ID_STD or CAN_ID_EXT
 * @param rtr_mode      RTR mode, can be CAN_RTR_DATA or CAN_RTR_REMOTE
 * @param len           Length of payload data
 * @param data          Payload data array, maximum of 8 bytes
 * @param rxCallback    Pointer to rx callback function
 * @param mutex_id_     Mutex handle for object
 * 
*/
class CANFrame {
public:
    CANFrame(uint32_t can_id, uint32_t id_type, uint32_t rtr_mode, uint32_t len):
        can_id(can_id), id_type(id_type), rtr_mode(rtr_mode), len(len)
    {
        mutex_id_ = osMutexNew(&mutex_attributes_);
    };

    CANFrame(uint32_t can_id, 
                uint32_t id_type, 
                uint32_t rtr_mode,
                uint32_t len,
                void (*rxCallback)(uint8_t data[])):
        can_id(can_id), id_type(id_type), rtr_mode(rtr_mode), len(len), rxCallback(rxCallback)
    {
        mutex_id_ = osMutexNew(&mutex_attributes_);
    };

    void LoadData(uint8_t data[], uint32_t len) {
        this->len = len;
        etl::mem_copy(&data[0], len, &this->data[0]);
    }

    uint32_t            can_id;             /* CAN ID, can be standard or extended */
    uint32_t            id_type;            /* CAN ID type, 0 if standard ID, 4 if extended ID */
    uint32_t            rtr_mode;           /* RTR mode, 0 if not RTR message, 2 if RTR */
    uint32_t            len;                /* payload data length */
    uint8_t             data[8];            /* payload data array, maximum of 8 bytes */
    void (*rxCallback)(uint8_t data[]);     /* pointer to rx callback function */
    osMutexId_t         mutex_id_;          /* mutex id for message */

private:
    StaticSemaphore_t   mutex_control_block_;
    const osMutexAttr_t mutex_attributes_ = {
        .name = "CANFrame Mutex",
        .attr_bits = osMutexRecursive,
        .cb_mem = &mutex_control_block_,
        .cb_size = sizeof(mutex_control_block_),
    };
};

/**
 * Class for controlling a single HAL CAN device.
 * 
 * Sends and receives CAN messages on a single CAN peripheral.
 * Spawns two threads, one for handling Tx and one for handling Rx.
*/
class CANDevice {
    friend class CANController;
public:
    CANDevice(CAN_HandleTypeDef *hcan) { hcan_ = hcan; }
    HAL_StatusTypeDef Start();
    HAL_StatusTypeDef Send(CANFrame *msg);
    uint32_t GetStatus();
    void SetRxFlag();
    osThreadId_t tx_task_id_;
    osThreadId_t rx_task_id_;

protected:
    CAN_HandleTypeDef *hcan_;
    etl::map<uint32_t, CANFrame*, MAX_RX_MSGS>* rx_messages_;
    osMessageQueueId_t tx_queue_ = osMessageQueueNew(TX_QUEUE_SIZE, sizeof(CANFrame*), NULL);
    etl::atomic<uint32_t> status_;      // TODO: handle this
    osEventFlagsId_t rx_event_flag_ = osEventFlagsNew(NULL);

    /* Tx thread definitions */
    uint32_t tx_task_buffer[128];   // Size can be much smaller w/o logging
    StaticTask_t tx_task_control_block_;
    const osThreadAttr_t tx_task_attributes_ = {
        .name = "CAN Tx Task",
        .attr_bits = osThreadDetached,
        .cb_mem = &tx_task_control_block_,
        .cb_size = sizeof(tx_task_control_block_),
        .stack_mem = &tx_task_buffer[0],
        .stack_size = sizeof(tx_task_buffer),
        .priority = (osPriority_t) THREAD_PRIORITY,
        .tz_module = 0,
        .reserved = 0,
    };

    /* Rx thread definitions */
    uint32_t rx_task_buffer[128];   // Size can be much smaller w/o logging
    StaticTask_t rx_task_control_block_;
    const osThreadAttr_t rx_task_attributes_ = {
        .name = "CAN Rx Task",
        .attr_bits = osThreadDetached,
        .cb_mem = &rx_task_control_block_,
        .cb_size = sizeof(rx_task_control_block_),
        .stack_mem = &rx_task_buffer[0],
        .stack_size = sizeof(rx_task_buffer),
        .priority = (osPriority_t) THREAD_PRIORITY,
        .tz_module = 0,
        .reserved = 0,
    };

    /* Tx thread function */
    void HandleTx(void* argument);

    /* Rx thread function */
    void HandleRx(void* argument);

    /* Tx timeout handler thread */
    void HandleTxTimeout();     // TODO: Test reset functionality
};

/**
 * Class for controlling CAN devices and messages.
*/
class CANController {
public:
    static HAL_StatusTypeDef AddDevice(CANDevice *device);
    static HAL_StatusTypeDef AddRxMessage(CANFrame *msg);
    static HAL_StatusTypeDef AddRxMessages(CANFrame *msg[], uint32_t num_msgs);
    static HAL_StatusTypeDef AddFilterAll();
    static HAL_StatusTypeDef AddFilterId(uint32_t can_id, uint32_t id_type, uint32_t rtr_mode, uint32_t priority);      // TODO:
    static HAL_StatusTypeDef AddFilterIdRange(uint32_t can_id, uint32_t range, uint32_t id_type, uint32_t rtr_mode, uint32_t priority); // TODO
    static HAL_StatusTypeDef Start();
    static HAL_StatusTypeDef Send(CANFrame *msg);
    static HAL_StatusTypeDef SendOnDevice(CANDevice *device, CANFrame *msg);
    static HAL_StatusTypeDef GetMessage(uint32_t can_id, CANFrame *msg);
    static HAL_StatusTypeDef GetDeviceStatus(CANDevice *device);    // TODO
    static void RxCallback(CAN_HandleTypeDef *hcan);
protected:
    static inline etl::vector<CANDevice*, 3> devices_;
    static inline etl::map<uint32_t, CANFrame*, MAX_RX_MSGS> rx_messages_;
    static inline uint32_t num_msgs_ = 0;
    static inline etl::deque<CAN_FilterTypeDef, NUM_FILTER_BANKS*2> filters_;
};

#endif  /* SG_CAN_HPP_ */