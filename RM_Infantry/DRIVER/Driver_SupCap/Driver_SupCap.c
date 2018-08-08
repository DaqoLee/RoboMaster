#include "Driver_SupCap.h"
uint8_t Sup_Cap,CAP_OFF=1;
/*
 0λ------------оƬʹ��״̬         1Ϊʹ��
 1λ------------��ѹ�������         1Ϊ��ѹ
 2λ------------����������         1Ϊ����
 3λ------------���������ü������  1Ϊ2A���
 4λ------------����Դ�������        1Ϊ������������̹��� 
*/
void Init_SupCap(void)
{
		Read_Cap_State();
}

/*
	��ȡ��������״̬
*/
void Read_Cap_State(void)
{
		uint8_t capParam=0;
		capParam|=((~CAP_READ_E)<<0)&0x01;
		capParam|=((~CAP_READ_V)<<1)&0x06;
		capParam|=CAP_READ_C<<3;
		capParam|=CAP_READ_S<<4;
		Sup_Cap=capParam;
}
/*
	��������ģʽ���������ݳ������Զ��л��������ݹ���
*/
void Cap_Mode(void)
{
	static uint8_t Flag1=0;
	if((DBUS_CheckPush(KEY_R)&&DBUS_CheckPush(KEY_SHIFT)))
	{
		if(Flag1)
		{
			CAP_OFF=!CAP_OFF;
			Flag1=0;
		}
	}
    else 
	{	
		Flag1=1;
	}
if(CAP_OFF)
{
	  CAP_CHARGING(unenable);
  	HAL_GPIO_WritePin(LED_8_GPIO_Port,LED_8_Pin,GPIO_PIN_SET);
}
else
{
		CAP_CHARGING(enable);
	  HAL_GPIO_WritePin(LED_8_GPIO_Port,LED_8_Pin,GPIO_PIN_RESET);
}

		if(Sup_Cap&0x01)
		{
				if(Sup_Cap&0x02&&Sup_Cap&0x10)//��ѹ���̵�Դ�лص�ع���
				{
					  HAL_GPIO_WritePin(LED_7_GPIO_Port,LED_7_Pin,GPIO_PIN_SET);
						CAP_POWER_SWITCH(battery_supply);
						CAP_SET_CURRENT(_05A_current);
				}
			  if(Sup_Cap&0x04&&(Sup_Cap&0x10)==0x00)
				{
					  HAL_GPIO_WritePin(LED_7_GPIO_Port,LED_7_Pin,GPIO_PIN_RESET);
						CAP_POWER_SWITCH(cap_supply);
						CAP_SET_CURRENT(_2A_current);
				}
		}
		else//ʧЧ������ʱ���̵�Դ�лص��������
		{
				CAP_POWER_SWITCH(battery_supply);
				CAP_SET_CURRENT(_05A_current);
		}
}
/*
	����ģʽ�����������ݳ���������0.5A���
*/
void Normal_Mode(void)
{
		CAP_CHARGING(enable);
		CAP_POWER_SWITCH(battery_supply);
		CAP_SET_CURRENT(_05A_current);
}
/*
	�������ݳ�����͹رճ������Լ�С����
*/
void Cap_Casual(void)//����ģʽ
{
		////////////�������ݳ��
//		CAP_CHARGING(enable);			
		if(Sup_Cap&0x01)
		{
				CAP_POWER_SWITCH(cap_supply);
				if((Sup_Cap&0x04)==0)
				{
						
						CAP_SET_CURRENT(_2A_current);
				}
				else
				{
						CAP_CHARGING(unenable);
						CAP_SET_CURRENT(_05A_current);
				}
		}
		else//��ʹ��оƬʱ�лص�ع���
		{
				CAP_POWER_SWITCH(battery_supply);
		}
}
