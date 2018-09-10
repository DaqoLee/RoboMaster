#include "Ctrl_Frict.h"
uint16_t FRICT_SPEED=800;//Ħ�����ٶ�
uint8_t  Frict_OFF=1;
 /*
  * @brief Ħ�����뼤���������
  * @param None
  * @retval None
  */
void Frict_Param_Set(void)//Ħ����
{
	if((!Frict_OFF||(DBUS_ReceiveData.switch_right==3&&DBUS_ReceiveData.switch_left!=1))&&(Game_Mode!=SUPPLY)&&Control_Mode!=Ctrl_OFF)
	{//Ħ�����ٶȻ���������ȥ�������ذ�����̫��ᵼ�¹�������
		TIM1->CCR1=TIM1->CCR1>=1000+FRICT_SPEED?1000+FRICT_SPEED:TIM1->CCR1+2;
		TIM1->CCR2=TIM1->CCR2>=1000+FRICT_SPEED?1000+FRICT_SPEED:TIM1->CCR2+2;
		TIM1->CCR3=TIM1->CCR3>=1000+FRICT_SPEED?1000+FRICT_SPEED:TIM1->CCR3+2;
		TIM1->CCR4=TIM1->CCR4>=1000+FRICT_SPEED?1000+FRICT_SPEED:TIM1->CCR4+2;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);//������	
	}
	else
	{
		TIM1->CCR1=1000;
		TIM1->CCR2=1000;
		TIM1->CCR3=1000;
		TIM1->CCR4=1000;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);//�ؼ���	
	}
	
}


