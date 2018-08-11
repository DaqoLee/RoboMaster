#include "Task_user.h"
#include "BSP_GPIO.h"
#include "stm32f4xx_hal.h"
TaskHandle_t StartTask_Handler;
TaskHandle_t LED0Task_Handler;
TaskHandle_t LED1Task_Handler;
TaskHandle_t FLOATTask_Handler;
TaskHandle_t LED_TaskHandler;
TaskHandle_t KEY_TaskHandler;
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();          
    
    xTaskCreate(led0_task,     	
				"led0_task",   	
			    50, 
                NULL,				
                6,	
                &LED0Task_Handler);   
   
    xTaskCreate(led1_task,     
                "led1_task",   
                50, 
                NULL,
                5,
                &LED1Task_Handler);        
   
    xTaskCreate(float_task,     
                "float_task",   
                128, 
                NULL,
                4,
                &FLOATTask_Handler);
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
				
    vTaskDelete(StartTask_Handler); 
    taskEXIT_CRITICAL();            
}


void led0_task(void *pvParameters)
{
    while(1)
    {
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        vTaskDelay(500);
    }
}     

void led1_task(void *pvParameters)
{
    while(1)
    {
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        vTaskDelay(50);
    }
}

void LED_Task(void *pvParameters)
{
	static uint8_t i=1;
	 while(1)
    {

		for(;i<17;i++)
		{
		   if(i>8)
				GPIOG->BSRR=1<<(i+8);
		   else
				GPIOG->BSRR=1<<i;
           vTaskDelay(50);
		}
		i=1;
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
			vTaskSuspend(LED0Task_Handler);
			vTaskSuspend(LED1Task_Handler);
			
		}
		else
		{
			vTaskResume(LED_TaskHandler);
			vTaskResume(LED0Task_Handler);
			vTaskResume(LED1Task_Handler);
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

