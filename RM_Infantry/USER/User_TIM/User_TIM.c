#include "User_TIM.h"
#include "tim.h"
void TIM_Init()
{

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//TIM1 ͨ��1ʹ�ܣ�Ħ���֣�
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//TIM1 ͨ��2ʹ�ܣ�Ħ���֣�
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//TIM1 ͨ��1ʹ�ܣ�Ħ���֣�
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//TIM1 ͨ��2ʹ�ܣ�Ħ���֣�
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim6);//��ʱ��6�ж�ʹ�� 1  ms
  HAL_TIM_Base_Start_IT(&htim7);//��ʱ��7�ж�ʹ�� 10 ms
	
}

