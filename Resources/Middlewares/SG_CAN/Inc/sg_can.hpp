/*
 * CANMessage.hpp
 *
 *  Created on: July 10, 2023
 *      Author: Matthew Shen
 */

#ifndef SG_CAN_HPP_
#define SG_CAN_HPP_

#define STM32L4xx       /* Development only, remove in final project */
#define CAN1


#include "stm32l4xx_hal_can.h"
#include <stdint.h>
#include <etl/map.h>

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

class CANManager {
public:
    CANManager() = default;
    CANManager(CAN_HandleTypeDef* hcan);
    ~CANManager() = default;
    int32_t Init();
    int32_t Send(SolarGators::CANMessage* msg);
    int32_t AddRxMessage(SolarGators::CANMessage* msg);
    int32_t AddRxMessages(SolarGators::CANMessage* msg[], uint32_t numMsgs);
    int32_t AddInterface(CAN_HandleTypeDef* hcan);
    void HandleReceive();
protected:
    uint32_t numMsgs = 0;
    uint32_t numInterfaces = 0;
    CAN_HandleTypeDef* interfaces[2] = {nullptr, nullptr};
    ::etl::map<uint32_t, SolarGators::CANMessage*, 64> rxMessages;
};

}   /* namespace SolarGators */

#endif  /* SG_CAN_HPP_ */


SolarGators::CANManager::CANManager(CAN_HandleTypeDef* hcan) {
    interfaces[0] = hcan;
    numInterfaces = 1;
}

int32_t SolarGators::CANManager::Init() {
    for (uint32_t i = 0; i < numInterfaces; i++)
        HAL_CAN_Start(interfaces[i]);
}

int32_t SolarGators::CANManager::Send(SolarGators::CANMessage* msg) {
    // Wait for one interface to become free

    CAN_TxHeaderTypeDef hal_can_header = {
        .StdId = msg->can_id,
        .ExtId = msg->can_id,
        .IDE = msg->id_type,
        .RTR = CAN_RTR_DATA,
        .DLC = msg->data_length,
    };

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(interfaces[0], &hal_can_header, msg->data, &tx_mailbox);

    return 0;
}

int32_t SolarGators::CANManager::AddRxMessage(SolarGators::CANMessage* msg) {
    rxMessages.insert(etl::make_pair(msg->can_id, msg));
    return 0;
}


// Put in user code
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    
}