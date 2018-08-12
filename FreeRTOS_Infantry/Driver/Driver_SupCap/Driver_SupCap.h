#ifndef __DRIVER_SUPCAP_H
#define __DRIVER_SUPCAP_H
#include "control.h"

typedef enum
{
		enable					=0x01,//��ʾ�Ƿ���ʹ�ܳ��
		unenable				,	//��ʾ��ʹ�ܳ��оƬ			
		cap_supply			,//��ʾ�������ݹ��磬Ĭ���ǵ�Դ����
		battery_supply  ,//��ظ����̹���
		_2A_current			,//��ʾ����2A�����Գ������ݳ��
		_05A_current		,//��ʾ����0.5A�����Գ������ݳ��
		under_voltage		,//��ʾ�������ݵ�ѹ����78%
		charge_complete ,//��ʾ�������ݵ�ѹ����96%
		unenable_cap		, //��������ʧЧ��ʹ�ó�������ģ��
		enable_cap        //���ó�������ģ��
}Sup_Cap_State;

/*
	PA3--------------ʹ�ܳ��оƬ����
	PF10-------------�����л�����
	PA5--------------������������
	PC5--------------����������
	PC4--------------��ѹ�������
*/

///////////////////////ʹ�ܳ��оƬ////////////////////////////////////////////////////
#define CAP_CHARGING(x)  		if(x==enable||x==unenable){\
															if(x==enable)\
															{GPIOE->BSRR=1<<21;}\
															else{GPIOE->BSRR=1<<5;}	}															
///////////////////////�л���Դ////////////////////////////////////////////////////													
#define CAP_POWER_SWITCH(x)  if(x==battery_supply||x==cap_supply){\
															if(x==cap_supply)\
															{GPIOC->BSRR=1<<4;}\
															else{GPIOC->BSRR=1<<20;}}
/////////////////////////���õ���//////////////////////////////////////////////////
#define CAP_SET_CURRENT(x)  if(x==_2A_current||x==_05A_current){\
															if(x==_2A_current)\
															{GPIOE->BSRR=1<<6;}\
															else{GPIOE->BSRR=1<<22;}}
#define CAP_READ_E        (GPIOE->IDR&=1<<5)>>5								//��ʹ��оƬ����
#define CAP_READ_S        (GPIOC->IDR&=1<<4)>>4					     	//����Դ�л�����					
#define CAP_READ_C        (GPIOE->IDR&=1<<6)>>6 							//��������������
#define CAP_READ_V        (GPIOC->IDR&=3<<2)>>2 							//����ѹ��ֵ����			
void Init_SupCap(void);
void Read_Cap_State(void);
void Cap_Mode(void);
void Normal_Mode(void);
void Cap_Casual(void);
															
extern uint8_t Sup_Cap,CAP_OFF;
#endif

