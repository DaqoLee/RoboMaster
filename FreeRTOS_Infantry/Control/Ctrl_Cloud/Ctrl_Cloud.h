#ifndef __CTRL_CLOUD_H
#define __CTRL_CLOUD_H
#include "User_Code.h"

extern  CloudParam_Struct   CloudParam;
extern const float	Pitch_Min,Pitch_Max;
extern const float  Yaw_Max,Yaw_Min;
extern const float	Gyro_Yaw_Min,Gyro_Yaw_Max;

void Cloud_Param_Set(void);//ÔÆÌ¨
void Cloud_Current_Set(CAN_X_State CAN_X);
void M6623_PID_Set(pid_t* PID_Yaw, float Yaw_Real,float Yaw_Target,pid_t* PID_Pitch, float Pitch_Real,float Pitch_Target);
#endif


