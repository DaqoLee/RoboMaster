/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "SystemClock.h"
#include "BSP_GPIO.h"
#include "BSP_TIM.h"
#include "BSP_USART.h"
#include "BSP_CAN.h"
#include "Task_user.h"

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
  /* Configure the system clock */
	SystemClock_Config();
  /* Initialize all configured peripherals */
	GPIO_Init();
	TIM_Init();
	UART_Init();
    CAN_Init(&hcan1);
    CAN_Init(&hcan2);
  /* Initialize all configured peripherals */
	
    xTaskCreate(start_task,            //任务函数
                "start_task",          //任务名称
                128,       			   //任务堆栈大小
                NULL,                  //传递给任务函数的参数
                1,       			   //任务优先级
                &StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();             //开启任务调度
	while(1)	
	{
	
	}

}


 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
