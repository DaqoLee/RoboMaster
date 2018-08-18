/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "SystemClock.h"
#include "Task_user.h"

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
  /* Configure the system clock */
	SystemClock_Config();
	
    xTaskCreate(start_task,            
                "start_task",          
                512,       			   
                NULL,                 
                1,       			   
                &StartTask_Handler);               
    vTaskStartScheduler();            
	
	while(1);	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
