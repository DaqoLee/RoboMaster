#ifndef __USER_CODE_H
#define __USER_CODE_H
#include "User_typedefs.h"
#include "Ctrl_Chassis.h"
#include "Ctrl_Cloud.h"
#include "Ctrl_Frict.h"
#include "Ctrl_Rammer.h"
#include "Driver_Judge.h"
#include "Driver_DBUS.h"
#include "Driver_PID.h"
#include "BSP_GPIO.h"
#include "BSP_TIM.h"
#include "BSP_USART.h"
#include "BSP_CAN.h"
#include "BSP_NVIC.h"

#define INFANTRY          4    //1�ź�4�Ų����в���

#if  INFANTRY==4
#define MEDIAN_PITCH      2400 //Pith���м��е�Ƕ�
#define MEDIAN_YAW        4000 //Yaw���м��е�Ƕ�
#define MEDIAN_ROLL				10
#elif INFANTRY==1
#define MEDIAN_PITCH      4000 //Pith���м��е�Ƕ�
#define MEDIAN_YAW        4400 //Yaw���м��е�Ƕ�
#define MEDIAN_ROLL				10
#endif
#define PITCH_RANGE		   28  		//Pitch�Ƕȷ�Χ
#define YAW_RANGE		   180		//Yaw�Ƕȷ�Χ
#define	YAW_GYRO_RANGE	   200      //�����ǽǶ��޷�

#define RAMMER_NUM		   8		//���̿�����
#define REDUCTION_RATIO	   (36/1)	//����������ٱ�
#define RAMMER_ERROR	   16000	//����תһȦ��ƫ��

#define M2006_Xianfu  8000
#define M3508_Xianfu  8000
#define M6623_Xianfu  6000

void Filters(int16_t Val,float *Value,float Rate);
void PID_Init(void);
#endif

