#pragma once
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include <cstdlib>
#include <cstring>

class CAN
{
public:
	void Init(CAN_TypeDef* instance);
	void InitFilter();
	HAL_StatusTypeDef Transmit(const uint32_t ID, 
							   const uint8_t*const pData, 
							   const uint8_t len = 8);
		CAN_HandleTypeDef hcan;
	uint8_t data[8][8];
private:
	CanTxMsgTypeDef	TxMessage;
	CanRxMsgTypeDef RxMessage;
};

extern CAN can1, can2;