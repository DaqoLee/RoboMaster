#include "Ctrl_Chassis.h"


void Set_moto_current(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
{

	hcan->pTxMsg->StdId = ID;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = Current1 >> 8;
	hcan->pTxMsg->Data[1] = Current1;
	hcan->pTxMsg->Data[2] = Current2 >> 8;
	hcan->pTxMsg->Data[3] = Current2;
	hcan->pTxMsg->Data[4] = Current3 >> 8;
	hcan->pTxMsg->Data[5] = Current3 ;
	hcan->pTxMsg->Data[6] = Current4 >> 8;
	hcan->pTxMsg->Data[7] = Current4;
	
	
	xQueueSend(Queue_CanSend, hcan, 20);
	
}

