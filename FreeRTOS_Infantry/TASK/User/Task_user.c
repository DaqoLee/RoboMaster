#include "Task_User.h"
#include "stm32f4xx_hal.h"
#include "Task_Can.h"
#include "Task_Ctrl.h"
TaskHandle_t StartTask_Handler;
TaskHandle_t Control_Mode_TaskHandler;
TaskHandle_t Frame_Rate_TaskHandler;
TaskHandle_t Buzzer_TaskHandler;
TaskHandle_t Task_CanSendHandler;
TaskHandle_t Task_ControlHandler;

void start_task(void *pvParameters)
{
	static uint8_t i=1;
	
	GPIO_Init();
	NVIC_Init();
	TIM_Init();
	UART_Init();
	CAN_Init(&hcan1);
	CAN_Init(&hcan2);
	PID_Init();
	
	TIM12->CCR1=200;
	for(;i<4;i++)
	{
		TIM12->ARR=1999-200*i;
		vTaskDelay(150);
	}
	TIM12->ARR=0;
	
	vTaskDelay(500);//等待陀螺仪数据稳定
	Get_Target_Angle();
	
	Queue_CanSend=xQueueCreate(64, sizeof(CanSend_Type));
	
    taskENTER_CRITICAL();            

	xTaskCreate(Frame_Rate_Task,  //创建帧率统计任务   
                "Frame_Rate_Task",   
                128, 
                NULL,
                2,
                &Frame_Rate_TaskHandler);  
	
    xTaskCreate(Control_Mode_Task,  //创建控制模式任务   
                "Control_Mode_Task",   
                128, 
                NULL,
                4,
                &Control_Mode_TaskHandler);  
	xTaskCreate(Task_Control,     //创建控制任务
                "Task_Control",   
                256, 
                NULL,
                4,
                &Task_ControlHandler); 
				
	xTaskCreate(Task_CanSend, //创建CAN发送任务
                "Task_CanSend",   
                256, 
                NULL,
				4,
                &Task_CanSendHandler);  	
								
    vTaskDelete(StartTask_Handler); 
    taskEXIT_CRITICAL();            
}

void Buzzer_Task(void *pvParameters)
{
	static uint8_t i=1;
	TIM12->CCR1=200;
	 while(1)
    {
		vTaskDelay(2000);
		for(;i<8;i++)
		{
			TIM12->ARR=1999-100*i;
			vTaskDelay(150);
		}
		TIM12->ARR=0;
    }
}

void Frame_Rate_Task(void *pvParameters)
{
	 while(1)
    {
		Frame_Rate_Statistics();
		vTaskDelay(100);
    }
}


