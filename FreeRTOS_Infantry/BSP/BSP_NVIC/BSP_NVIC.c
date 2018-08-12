#include "BSP_NVIC.h"

void NVIC_Init(void)
{
	//TIM7
    HAL_NVIC_SetPriority(TIM7_IRQn,	   6, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
	//TIM6
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
	
	//CAN1
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn,2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	//CAN2
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn,3, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	
	
	//USART1
	HAL_NVIC_SetPriority(USART1_IRQn,  1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
	
	//USART2
    HAL_NVIC_SetPriority(USART2_IRQn,  10, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	
	//USART3
    HAL_NVIC_SetPriority(USART3_IRQn,  4, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn); 
	
	//USART6
    HAL_NVIC_SetPriority(USART6_IRQn,  6, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
	
	//UART7
    HAL_NVIC_SetPriority(UART7_IRQn,   8, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
	
	//UART8
    HAL_NVIC_SetPriority(UART8_IRQn,   9, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
}
