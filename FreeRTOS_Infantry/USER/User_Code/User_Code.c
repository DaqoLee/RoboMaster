#include "User_Code.h"

 /*
  * @brief PID参数初始化
  * @param None
  * @retval None
  */
void PID_Init(void)
{
/********************拨弹电机*********************输出限幅****积分限幅** P **** I *** D ****/
	PID_struct_init(&M2006.PID.Out,POSITION_PID, M2006_Xianfu,	500,	0.8f,	0,	2.8f);
	PID_struct_init(&M2006.PID.In,DELTA_PID,	 M2006_Xianfu,	500,	0.4f,	0,	   0);
	
/********************底盘电机*************************输出限幅****积分限幅** P ***** I **** D **/	
    PID_struct_init(&ChassisParam.LF.PID,DELTA_PID,	M3508_Xianfu,	800,	2.5f,	1.0f,	0);
	PID_struct_init(&ChassisParam.LB.PID,DELTA_PID,	M3508_Xianfu,	800,	2.5f,	1.0f,	0);
	PID_struct_init(&ChassisParam.RF.PID,DELTA_PID,	M3508_Xianfu,	800,	2.5f,	1.0f,	0);
	PID_struct_init(&ChassisParam.RB.PID,DELTA_PID,	M3508_Xianfu,	800,	2.5f,	1.0f,	0);
	
/********************云台电机*********************************输出限幅****积分限幅** P **** I *** D */
	PID_struct_init(&CloudParam.Pitch.PID.Out,POSITION_PID,	M6623_Xianfu,	2000,	6.5f,	0,	2.0f);
	PID_struct_init(&CloudParam.Pitch.PID.In,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	0,	   1);
	PID_struct_init(&CloudParam.Yaw.PID.Out,POSITION_PID,	M6623_Xianfu,	2000,	3.6f,	0,	4.6f);
	PID_struct_init(&CloudParam.Yaw.PID.In,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	0,	   1);

/*******************云台陀螺仪********************************************输出限幅****积分限幅** P ****** I ***** D **/
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.Out,POSITION_PID,	M6623_Xianfu,	500,	100,	  0,	1000);
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.In,POSITION_PID,	M6623_Xianfu,	1000,	1.2,	0.1,	   2);
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.Out,POSITION_PID,	M6623_Xianfu,	500,	120,	  0,	 100);
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.In ,POSITION_PID,	M6623_Xianfu,	2000,	0.8,	  0,	   1);

}
 /*
  * @brief 滤波函数，数据平滑处理
  * @param Val 滤波前，*Value滤波后
  * @retval None
  */
void Filters(int16_t Val,float *Value,float Rate)
{
	*Value+=Rate*(Val-*Value);
}

