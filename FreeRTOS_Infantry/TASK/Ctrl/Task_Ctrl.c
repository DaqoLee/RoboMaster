#include "Task_Ctrl.h"

void Task_Control(void *Parameters)
{
	portTickType CurrentControlTick = 0;
	
	while(1)
	{
		Frict_Param_Set();//Ħ����
		Rammer_Param_Set();//����
		Chassis_Param_Set();//����
		Cloud_Param_Set();//��̨
		
		vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);//5ms��ʱ
	}
}



void Control_Mode_Task(void *pvParameters)
{
	 while(1)
    {
		Control_Mode_Set();//ң��ģʽ1��2������ģʽ��ʧ�ܿ����л���
        vTaskDelay(50);

    }
}

