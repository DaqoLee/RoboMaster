#include "Task_Ctrl.h"

void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	
	while(1)
	{
		M2006_PID_Set();
		Set_moto_current(&hcan1,0x1FF,0,0,M2006.Target_Current,0);
		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);
	}
}



