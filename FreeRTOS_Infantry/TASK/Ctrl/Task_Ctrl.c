#include "Task_Ctrl.h"

void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	
	while(1)
	{
//		Frict_Param_Set();
		Chassis_Param_Set();
		Cloud_Param_Set();
//		Moto_Current_Set(&hcan1,0x200, ChassisParam.LB.Target_Current, ChassisParam.RB.Target_Current, \
//		ChassisParam.RF.Target_Current, ChassisParam.LF.Target_Current);
//		Moto_Current_Set(&hcan1,0x1FF, CloudParam.Yaw.Target_Current, CloudParam.Pitch.Target_Current, \
//		M2006.Target_Current, 0);
		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);
	}
}



