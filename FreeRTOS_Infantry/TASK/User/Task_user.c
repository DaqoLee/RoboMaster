#include "Task_User.h"
#include "stm32f4xx_hal.h"
#include "BSP_GPIO.h"
#include "BSP_TIM.h"
#include "BSP_USART.h"
#include "BSP_CAN.h"
#include "BSP_NVIC.h"
#include "Task_Can.h"
#include "Task_Ctrl.h"
#include "Driver_DBUS.h"
TaskHandle_t StartTask_Handler;
TaskHandle_t FLOATTask_Handler;
TaskHandle_t LED_TaskHandler;
TaskHandle_t KEY_TaskHandler;
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
	TIM12->CCR1=200;
	for(;i<4;i++)
	{
		TIM12->ARR=1999-200*i;
		vTaskDelay(150);
	}
	TIM12->ARR=0;
	Queue_CanSend=xQueueCreate(128, sizeof(CAN_HandleTypeDef));
	
    taskENTER_CRITICAL();            
   
    xTaskCreate(Buzzer_Task,     
                "Buzzer_Task",   
                128, 
                NULL,
                2,
                &Buzzer_TaskHandler);
    xTaskCreate(LED_Task,     
                "LED_Task",   
                128, 
                NULL,
                3,
                &LED_TaskHandler);  
	xTaskCreate(KEY_Task,     
                "KEY_Task",   
                128, 
                NULL,
                2,
                &KEY_TaskHandler);  	
	xTaskCreate(Task_CanSend,     
                "Task_CanSend",   
                300, 
                NULL,
								4,
                &Task_CanSendHandler);  	
	xTaskCreate(Task_Control,     
                "Task_Control",   
                128, 
                NULL,
                4,
                &Task_ControlHandler); 				
				
    vTaskDelete(StartTask_Handler); 
    taskEXIT_CRITICAL();            
}

void LED_Task(void *pvParameters)
{
	static uint8_t i=1;
	 while(1)
    {
		for(i=1;i<17;i++)
		{
		   if(i>8)
				GPIOG->BSRR=1<<(i+8);
		   else
				GPIOG->BSRR=1<<i;
		   HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
           vTaskDelay(40);
		}
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
		i=1;
    }
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

void KEY_Task(void *pvParameters)
{
	static uint8_t Flag1=1,Flag2=0;
	 while(1)
    {
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
		{
			vTaskDelay(20);
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)&&Flag1)
			{
				Flag1=0;
				Flag2=!Flag2;
			}
		}
		else
			Flag1=1;
		if(Flag2)
		{
			vTaskSuspend(LED_TaskHandler);
		}
		else
		{
			vTaskResume(LED_TaskHandler);
		}
    }
}

void float_task(void *pvParameters)
{
	static float float_num=0.00;
	while(1)
	{
		float_num+=0.01f;
		printf("%f\r\n",float_num);
        vTaskDelay(1000);
	}
}

