#include "Task_Ctrl.h"
#include "Ctrl_Chassis.h"
#include "Ctrl_Rammer.h"
void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	PID_Config();
	
	while(1)
	{
		M2006_PID_Set();
		Set_moto_current(&hcan1,0x1FF,0,0,M2006.Target_Current,0);
		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);
	}
}

void PID_Config(void)
{
		M2006.Target_Angle=50000;
		PID_struct_init(&M2006.PID.Out,POSITION_PID,6000,M2006.IntegralLimit,0.8f,0,2.8f);
		PID_struct_init(&M2006.PID.In,DELTA_PID,6000,M2006.IntegralLimit,0.4f,0,0);
}

