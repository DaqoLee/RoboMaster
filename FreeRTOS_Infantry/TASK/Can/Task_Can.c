#include "Task_Can.h"

xQueueHandle    Queue_CanSend; 

void Task_CanSend(void *Parameters)
{
	CanSend_Type   SendData;
	while(1)
	{
		xQueueReceive(Queue_CanSend, &SendData, portMAX_DELAY);
		if(SendData.CANx==CAN_1)
		{
			hcan1.pTxMsg=&SendData.SendCanTxMsg;
			HAL_CAN_Transmit(&hcan1, 100);
		}
		else if(SendData.CANx==CAN_2)
		{
			hcan2.pTxMsg=&SendData.SendCanTxMsg;
			HAL_CAN_Transmit(&hcan2, 100);
		}
	}

}

