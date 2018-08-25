#include "Task_Ctrl.h"

void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	
	while(1)
	{
		Frict_Param_Set();//摩擦轮
		Rammer_Param_Set();//拨弹
		Chassis_Param_Set();//底盘
		Cloud_Param_Set();//云台
		
		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);//5ms延时
	}
}



void Control_Mode_Task(void *pvParameters)
{
	 while(1)
    {
		Control_Mode_Set();//遥控模式1、2，键盘模式，失能控制切换。
        vTaskDelay(50);

    }
}

