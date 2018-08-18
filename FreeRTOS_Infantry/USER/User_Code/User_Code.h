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

#define INFANTRY          4    //1号和4号参数有差异

#if  INFANTRY==4
#define MEDIAN_PITCH      2400 //Pith轴中间机械角度
#define MEDIAN_YAW        4000 //Yaw轴中间机械角度
#define MEDIAN_ROLL				10
#elif INFANTRY==1
#define MEDIAN_PITCH      4000 //Pith轴中间机械角度
#define MEDIAN_YAW        4400 //Yaw轴中间机械角度
#define MEDIAN_ROLL				10
#endif
#define PITCH_RANGE		   28  		//Pitch角度范围
#define YAW_RANGE		   180		//Yaw角度范围
#define	YAW_GYRO_RANGE	   200      //陀螺仪角度限幅

#define RAMMER_NUM		   8		//拨盘孔数量
#define REDUCTION_RATIO	   (36/1)	//拨弹电机减速比
#define RAMMER_ERROR	   16000	//拨盘转一圈的偏差

#define M2006_Xianfu  8000
#define M3508_Xianfu  8000
#define M6623_Xianfu  6000

void Filters(int16_t Val,float *Value,float Rate);
void PID_Init(void);
#endif

