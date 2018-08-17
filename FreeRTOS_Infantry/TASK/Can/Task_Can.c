#include "Task_Can.h"

xQueueHandle    Queue_CanSend; 

void Task_CanSend(void *Parameters)
{
	CAN_HandleTypeDef Can;
	while(1)
	{
		xQueueReceive(Queue_CanSend, &Can, portMAX_DELAY);
		HAL_CAN_Transmit(&Can, 100);
	}


}

