#include "Task_Ctrl.h"

void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	
	while(1)
	{

		Control_Mode_Set();
		Frict_Param_Set();
		Chassis_Param_Set();

		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);
	}
}



