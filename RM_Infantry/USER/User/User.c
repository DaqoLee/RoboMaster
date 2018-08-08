#include "User.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Driver_DBUS.h"
#include "user_can.h"
#include "control.h"
#include "Ctrl_Cloud.h"
#include "Driver_Gyro.h"
#include "Driver_Hierogram.h"
 /*
  * @brief 电机的相关参数初始化
  * @param None
  * @retval None
  */
void Motor_Param_Init()
{
/********************* 底盘 M3508 电机参数初始化 *******************************/	
	ChassisParam.TargetVX=0;
	ChassisParam.TargetVY=0;
	
	ChassisParam.LF.KP=2.5f;
	ChassisParam.LF.KI=1.0f;
	ChassisParam.LF.KD=0;
	ChassisParam.LF.Target_Speed=0;
	ChassisParam.LF.IntegralLimit=800;
	PID_struct_init(&ChassisParam.LF.PID,DELTA_PID,M3508_Xianfu,ChassisParam.LF.IntegralLimit,\
									ChassisParam.LF.KP,ChassisParam.LF.KI,ChassisParam.LF.KD);
	
	ChassisParam.LB.KP=2.5f;
	ChassisParam.LB.KI=1.0f;
	ChassisParam.LB.KD=0;
	ChassisParam.LB.Target_Speed=0;
	ChassisParam.LB.IntegralLimit=800;
	PID_struct_init(&ChassisParam.LB.PID,DELTA_PID,M3508_Xianfu,ChassisParam.LB.IntegralLimit,\
									ChassisParam.LB.KP,ChassisParam.LB.KI,ChassisParam.LB.KD);
	
	ChassisParam.RF.KP=2.5f;
	ChassisParam.RF.KI=1.0f;
	ChassisParam.RF.KD=0;
	ChassisParam.RF.Target_Speed=0;
	ChassisParam.RF.IntegralLimit=800;
	PID_struct_init(&ChassisParam.RF.PID,DELTA_PID,M3508_Xianfu,ChassisParam.RF.IntegralLimit,\
									ChassisParam.RF.KP,ChassisParam.RF.KI,ChassisParam.RF.KD);
	
	ChassisParam.RB.KP=2.5f;
	ChassisParam.RB.KI=1.0f;
	ChassisParam.RB.KD=0;
	ChassisParam.RB.Target_Speed=0;
	ChassisParam.RB.IntegralLimit=800;
	PID_struct_init(&ChassisParam.RB.PID,DELTA_PID,M3508_Xianfu,ChassisParam.RB.IntegralLimit,\
									ChassisParam.RB.KP,ChassisParam.RB.KI,ChassisParam.RB.KD);
									
	PID_struct_init(&ChassisParam.Chassis_Gyro.Chassis_PID,POSITION_PID,4000,1000,3.5f,0,0);
/********************* 云台 M6623 电机参数初始化 *******************************/
  
	CloudParam.Pitch.KP=6.5;
	CloudParam.Pitch.KI=0;
	CloudParam.Pitch.KD=2;
	CloudParam.Pitch.IntegralLimit=2000;
	CloudParam.Pitch.Target_Angle=First_Pitch_Angle;
	PID_struct_init(&CloudParam.Pitch.PID.Out,POSITION_PID,M6623_Xianfu,CloudParam.Pitch.IntegralLimit,\
										  CloudParam.Pitch.KP,CloudParam.Pitch.KI,CloudParam.Pitch.KD);
	PID_struct_init(&CloudParam.Pitch.PID.In,POSITION_PID,6000,2000,0.8f,0,1);
	
	CloudParam.Yaw.KP=3.6;
	CloudParam.Yaw.KI=0;
	CloudParam.Yaw.KD=4.6;
	CloudParam.Yaw.IntegralLimit=1000;
	CloudParam.Yaw.Target_Angle=First_Yaw_Angle;
	PID_struct_init(&CloudParam.Yaw.PID.Out,POSITION_PID,M6623_Xianfu,CloudParam.Yaw.IntegralLimit,\
										    CloudParam.Yaw.KP,CloudParam.Yaw.KI,CloudParam.Yaw.KD);
	PID_struct_init(&CloudParam.Yaw.PID.In,POSITION_PID,6000,1000,1.2,0.1,2);
	

	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.Out,POSITION_PID,6000,500,100,0,1000);
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.In,POSITION_PID,6000,1000,1.2,0.1,2);
	
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.Out,POSITION_PID,6000,500,120,0,100);
  PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.In ,POSITION_PID,6000,2000,0.8,0,1);

  PID_struct_init(&Auto_Pitch_PID,POSITION_PID,6000,500,1,0,0);
	PID_struct_init(&Auto_Yaw_PID,POSITION_PID,6000,500,1,0,0);

/********************* 拨弹 M2006 电机参数初始化 *******************************/	
	M2006.Last_Angle=M2006.Real_Angle;	
	
	M2006.Angle_KP=0.8f;
	M2006.Angle_KI=0;
	M2006.Angle_KD=2.8f;
	PID_struct_init(&M2006.PID.Out,POSITION_PID,M2006_Xianfu,M2006.IntegralLimit,\
										M2006.Angle_KP,M2006.Angle_KI,M2006.Angle_KD);
	
	M2006.Speed_KP=0.4f;
	M2006.Speed_KI=0.016f;
	M2006.Speed_KD=0;
	PID_struct_init(&M2006.PID.In,DELTA_PID,M2006_Xianfu,M2006.IntegralLimit,\
										M2006.Speed_KP,M2006.Speed_KI,M2006.Speed_KD);
	
	M2006.IntegralLimit=120;
	M2006.Target_Angle =0;
	M2006.Target_Speed =1500;
	M2006.All_error_Angle=0;
	M2006.Error_Angle=0;
	
	Set_moto_current(&hcan1,0x200, 0, 0, 0, 0);//电机ID 0x201-0x204
	Set_moto_current(&hcan1,0x1FF, 0, 0, 0, 0);//电机ID 0x205-0x208	
	
}

void Bell_Init()
{	
	uint8_t i=1;
	TIM12->CCR1=200;
	for(;i<4;i++)
	{
		TIM12->ARR=1999-200*i;
		HAL_Delay(150);
	}
	TIM12->ARR=0;
}



