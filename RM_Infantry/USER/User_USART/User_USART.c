#include "User_USART.h"
#include "usart.h"
#include "Driver_Niming.h"
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
 
 }
 
 void UART_Init()
 {
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE );//����1�����ж�ʹ��
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE );//����2�����ж�ʹ�� 
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE );//����3�����ж�ʹ��
  __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE );//����6�����ж�ʹ��
  __HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE );//����7�����ж�ʹ��
  __HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE );//����8�����ж�ʹ��
	 
	 Init_Ninming(&huart2);//������λ����ʼ��
 }
 
