#include "Task_Ctrl.h"
#include "Ctrl_Chassis.h"

void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	while(1)
	{
		Set_moto_current(&hcan1,0x1FF,0,0,500,0);
		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);
	}
}

