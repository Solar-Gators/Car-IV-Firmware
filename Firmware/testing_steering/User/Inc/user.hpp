/*
 * user.hpp
 *
 *  Created on: Apr 5, 2023
 *      Author:  Anthony Kfoury, Braden Azis
 */

#ifndef USER_HPP_
#define USER_HPP_

#include "main.h"
#include "logger.hpp"
#include "UI.hpp"

void UI_task(void* argument);
void CAN_Init();
void Button_Init();
float Efficiency();
float Speed(uint8_t data[]);
void UMotors(uint8_t data[]);
void UBMSRx0(uint8_t data[]);
void UBMSRx2(uint8_t data[]);
void UBMSRx4(uint8_t data[]);
void Button0();
void Button1();
void Button2();
void Button3();
void Button4();
void Button5();
void Button6();
void HornReleased();
void HornPressed();
void PushToTalkPressed();
void PushToTalkReleased();


enum UIMsg{
	BMSRx0       = 0,
	BMSRx2       = 1,
	MotorRx0     = 2,
	LeftTurn     = 3,
	RightTurn    = 4,
	RightAndLeft = 5,
	BPS          = 6,
	CruiseEnable = 7,
	EcoEnabled   = 8,
	HeadLights   = 9,
	Horn         = 10,
	PushToTalk   = 11,
	BMSRx4       = 12
};

/**Type for UI messages*/
class UIQueue_Msg{
	public:
	uint8_t MsgType;
	uint8_t *data;
    UIQueue_Msg(UIMsg type, uint8_t *d){
        MsgType = type;
        data = d;
    }
	~UIQueue_Msg(){
		delete data;
	}
};
#endif
