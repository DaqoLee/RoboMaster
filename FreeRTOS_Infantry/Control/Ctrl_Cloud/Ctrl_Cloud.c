#include "Ctrl_Cloud.h"

CloudParam_Struct   CloudParam;

const float	Pitch_Min=((8192.0f*(PITCH_RANGE/360.0f))/2+0.5f),Pitch_Max=(8192.0f-Pitch_Min+0.5f);//根据角度限制范围确定Pitch最小和最大机械角度
const float Yaw_Min=((8192.0f*(YAW_RANGE/360.0f))/2+0.5f),Yaw_Max=(8192.0f-Yaw_Min+0.5f);
const float	Gyro_Yaw_Min=YAW_GYRO_RANGE/2,Gyro_Yaw_Max=360-Gyro_Yaw_Min;


 /*
  * @brief 云台电机和陀螺仪相关参数的处理，云台电机的绝对是编码器（0~8191），陀螺仪安装位置影响，Pitch轴真实参数用Roll替换
  * @param None
  * @retval None
  */

void Cloud_Param_Set(void)//云台
{	

/**************************************把目标值限制在0~8191**************************************/	
	CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Target_Angle>8191?CloudParam.Pitch.Target_Angle-8191:CloudParam.Pitch.Target_Angle;
	CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Target_Angle<0?CloudParam.Pitch.Target_Angle+8191:CloudParam.Pitch.Target_Angle;
	
	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle>8191?CloudParam.Yaw.Target_Angle-8191:CloudParam.Yaw.Target_Angle;
	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle<0?CloudParam.Yaw.Target_Angle+8191:CloudParam.Yaw.Target_Angle;
	
	CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>360?CloudParam.Cloud_Gyro.Target_Yaw-360:CloudParam.Cloud_Gyro.Target_Yaw;
	CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<0?CloudParam.Cloud_Gyro.Target_Yaw+360:CloudParam.Cloud_Gyro.Target_Yaw;
	
	CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Target_Roll>10?10:CloudParam.Cloud_Gyro.Target_Roll;
	CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Target_Roll<-15?-15:CloudParam.Cloud_Gyro.Target_Roll;
/***********************************Yaw轴限制在180°和过零处理************************************/	
	if(MEDIAN_YAW>Yaw_Min&&MEDIAN_YAW<Yaw_Max)//中间值在最大值和最小值之间，不会经过零点
	{
		
	  	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle>MEDIAN_YAW+Yaw_Min?MEDIAN_YAW+Yaw_Min:CloudParam.Yaw.Target_Angle;
	    CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle<MEDIAN_YAW-Yaw_Min?MEDIAN_YAW-Yaw_Min:CloudParam.Yaw.Target_Angle;
		
	}
	
	else if(MEDIAN_YAW<Yaw_Min)//中间值小于最小值，过零处理
	{
		CloudParam.Yaw.Target_Angle=(CloudParam.Yaw.Target_Angle<(MEDIAN_YAW+4096))&&\
		(CloudParam.Yaw.Target_Angle>(MEDIAN_YAW+Yaw_Min))?MEDIAN_YAW+Yaw_Min:CloudParam.Yaw.Target_Angle;
		CloudParam.Yaw.Target_Angle=(CloudParam.Yaw.Target_Angle>(MEDIAN_YAW+4096))&&\
		(CloudParam.Yaw.Target_Angle<(MEDIAN_YAW+Yaw_Max))?MEDIAN_YAW+Yaw_Max:CloudParam.Yaw.Target_Angle;
	}
	else
	{
		CloudParam.Yaw.Target_Angle=(CloudParam.Yaw.Target_Angle>(MEDIAN_YAW-4096))&&\
		(CloudParam.Yaw.Target_Angle<(MEDIAN_YAW-Yaw_Min))?MEDIAN_YAW-Yaw_Min:CloudParam.Yaw.Target_Angle;
		CloudParam.Yaw.Target_Angle=(CloudParam.Yaw.Target_Angle<(MEDIAN_YAW-4096))&&\
		(CloudParam.Yaw.Target_Angle>(MEDIAN_YAW-Yaw_Max))?MEDIAN_YAW-Yaw_Max:CloudParam.Yaw.Target_Angle;
	}
	
/***********************************Pitch轴限制在60°和过零处理************************************/	
	if(MEDIAN_PITCH>Pitch_Min&&MEDIAN_PITCH<Pitch_Max)
	{
		if(Game_Mode!=SUPPLY)//补给模式时解除角度限制
		{
			CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Target_Angle>MEDIAN_PITCH+Pitch_Min?MEDIAN_PITCH+(int)Pitch_Min:CloudParam.Pitch.Target_Angle;
	        CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Target_Angle<MEDIAN_PITCH-Pitch_Min?MEDIAN_PITCH-(int)Pitch_Min:CloudParam.Pitch.Target_Angle;
		}
	}
	
	else if(MEDIAN_PITCH<Pitch_Min)
	{
		CloudParam.Pitch.Target_Angle=(CloudParam.Pitch.Target_Angle<(MEDIAN_PITCH+4096))&&\
		(CloudParam.Pitch.Target_Angle>(MEDIAN_PITCH+Pitch_Min))?MEDIAN_PITCH+Pitch_Min:CloudParam.Pitch.Target_Angle;
		CloudParam.Pitch.Target_Angle=(CloudParam.Pitch.Target_Angle>(MEDIAN_PITCH+4096))&&\
		(CloudParam.Pitch.Target_Angle<(MEDIAN_PITCH+Pitch_Max))?MEDIAN_PITCH+Pitch_Max:CloudParam.Pitch.Target_Angle;
	}
	else
	{
		CloudParam.Pitch.Target_Angle=(CloudParam.Pitch.Target_Angle>(MEDIAN_PITCH-4096))&&\
		(CloudParam.Pitch.Target_Angle<(MEDIAN_PITCH-Pitch_Min))?MEDIAN_PITCH-Pitch_Min :CloudParam.Pitch.Target_Angle;
		CloudParam.Pitch.Target_Angle=(CloudParam.Pitch.Target_Angle<(MEDIAN_PITCH-4096))&&\
		(CloudParam.Pitch.Target_Angle>(MEDIAN_PITCH-Pitch_Max))?MEDIAN_PITCH-Pitch_Max:CloudParam.Pitch.Target_Angle;
	}
	
		switch(Control_Mode)//右键在中间为遥控模式，默认是模式1，底盘跟随
		{
/************************************************************************************************************************/
		case    Remote_1://遥控模式(跟随云台)
			
					if(!CloudParam.Cloud_Gyro.Offline)
					{
						CloudParam.Cloud_Gyro.Target_Yaw=ABS(DBUS_ReceiveData.ch1)>20?CloudParam.Cloud_Gyro.Target_Yaw+DBUS_ReceiveData.ch1*0.005:CloudParam.Cloud_Gyro.Target_Yaw;
						CloudParam.Cloud_Gyro.Target_Roll=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Cloud_Gyro.Target_Roll-DBUS_ReceiveData.ch2*0.005:CloudParam.Cloud_Gyro.Target_Roll;
						CloudParam.Pitch.Target_Angle=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Pitch.Target_Angle+DBUS_ReceiveData.ch2*0.05f:CloudParam.Pitch.Target_Angle;

						if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
								||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
						{
							CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
						}


					if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
						||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
					{
						CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
					}

						CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>360?CloudParam.Cloud_Gyro.Target_Yaw-360:CloudParam.Cloud_Gyro.Target_Yaw;
						CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<0?CloudParam.Cloud_Gyro.Target_Yaw+360:CloudParam.Cloud_Gyro.Target_Yaw;

						M6623_PID_Set(&CloudParam.Cloud_Gyro.Yaw_PID.Out,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
						&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
					
						CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
					}
					else
					{
						CloudParam.Pitch.Target_Angle=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Pitch.Target_Angle-DBUS_ReceiveData.ch2*0.02f:CloudParam.Pitch.Target_Angle;
						CloudParam.Yaw.Target_Angle=ABS(DBUS_ReceiveData.ch1)>20?CloudParam.Yaw.Target_Angle-DBUS_ReceiveData.ch1*0.02f:CloudParam.Yaw.Target_Angle;
						
						M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,CloudParam.Yaw.Target_Angle,\
						&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
						
						CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
						CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
					}


				break ;
				
/************************************************************************************************************************/				
		case    Remote_2://遥控模式(不跟随云台)
			
						CloudParam.Pitch.Target_Angle=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Pitch.Target_Angle-DBUS_ReceiveData.ch2*0.02f:CloudParam.Pitch.Target_Angle;
						
						M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,MEDIAN_YAW,\
						&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
						
						CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
						CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
		
				break ;
/************************************************************************************************************************/
		case    Keyboard://键鼠模式
			
				PID_REST(Game_Mode);
				switch(Game_Mode)
				{
/************************************************************************************************************************/
					case    COMMON://一般
								if(!CloudParam.Cloud_Gyro.Offline)
								{
									if(DBUS_CheckPush(KEY_CTRL))
									{
										CloudParam.Cloud_Gyro.Target_Yaw+=0.01f*DBUS_ReceiveData.mouse.x;
										CloudParam.Cloud_Gyro.Target_Roll+=0.01f*DBUS_ReceiveData.mouse.y;
										CloudParam.Pitch.Target_Angle-=0.2f*DBUS_ReceiveData.mouse.y;
									}
									else 
									{
										//限制鼠标速度
										DBUS_ReceiveData.mouse.x=DBUS_ReceiveData.mouse.x>200?200:DBUS_ReceiveData.mouse.x;
										DBUS_ReceiveData.mouse.x=DBUS_ReceiveData.mouse.x<-200?-200:DBUS_ReceiveData.mouse.x;
										
										DBUS_ReceiveData.mouse.y=DBUS_ReceiveData.mouse.y>200?200:DBUS_ReceiveData.mouse.y;
										DBUS_ReceiveData.mouse.y=DBUS_ReceiveData.mouse.y<-200?-200:DBUS_ReceiveData.mouse.y;
										
										CloudParam.Cloud_Gyro.Target_Yaw+=0.04f*DBUS_ReceiveData.mouse.x;
										CloudParam.Cloud_Gyro.Target_Roll+=0.02f*DBUS_ReceiveData.mouse.y;
										CloudParam.Pitch.Target_Angle-=0.8f*DBUS_ReceiveData.mouse.y;
										
									}

									if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
										||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
									{
										CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
									}

									
									CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>360?CloudParam.Cloud_Gyro.Target_Yaw-360:CloudParam.Cloud_Gyro.Target_Yaw;
									CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<0?CloudParam.Cloud_Gyro.Target_Yaw+360:CloudParam.Cloud_Gyro.Target_Yaw;
									
									M6623_PID_Set(&CloudParam.Cloud_Gyro.Yaw_PID.Out,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
									&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
									CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
									
								}	
								else
								{
									CloudParam.Pitch.Target_Angle-=0.5f*DBUS_ReceiveData.mouse.y;
									CloudParam.Yaw.Target_Angle-=0.5f*DBUS_ReceiveData.mouse.x;
									//云台PID计算
									M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,CloudParam.Yaw.Target_Angle,\
									&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
									
									CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;//真实值作为目标值，切换时保证位置不变
									CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
								}
								break ;
					case    SUPPLY://一般
						
								M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,MEDIAN_YAW,\
								&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,MEDIAN_PITCH-500);
								
								CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
								CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
							break ;
					case    HIEROGRAM://一般
						
							break ;
				}
				break ;
/************************************************************************************************************************/
		case    Ctrl_OFF://失能控制
					CloudParam.Pitch.Target_Current=0;
					CloudParam.Yaw.Target_Current=0;
		
				break ;
/************************************************************************************************************************/
		}
		Cloud_Current_Set(CAN_1);
}


 /*
  * @brief 云台电机PID计算
  * @param Yaw轴与Pitch轴的PID结构体，实际值与目标值。
  * @retval None
  */
void M6623_PID_Set(pid_t* PID_Yaw, float Yaw_Real,float Yaw_Target,pid_t* PID_Pitch, float Pitch_Real,float Pitch_Target)
{
	float Current=0,Current1=0;
	//云台Pitch轴
	Current=pid_calc(PID_Pitch,Pitch_Real,Pitch_Target);//外环PID计算
	
	if(PID_Pitch==&CloudParam.Pitch.PID.Out)//判断外环类型
	{
		Current1=pid_calc(&CloudParam.Pitch.PID.In,CloudParam.Cloud_Gyro.Gyr_Y,Current);//内环PID计算
		if(!CloudParam.Cloud_Gyro.Offline)
			CloudParam.Pitch.Target_Current=-Current1;
		else
			CloudParam.Pitch.Target_Current=-Current;
	}
	else if(PID_Pitch==&CloudParam.Cloud_Gyro.Pitch_PID.Out)
	{
		Current1=pid_calc(&CloudParam.Cloud_Gyro.Pitch_PID.In,CloudParam.Cloud_Gyro.Gyr_Y,Current);
		CloudParam.Pitch.Target_Current=Current;
	}

	
	//云台Yaw轴
	Current=pid_calc(PID_Yaw,Yaw_Real,Yaw_Target);
	if(PID_Yaw==&CloudParam.Yaw.PID.Out)
	{
		Current1=pid_calc(&CloudParam.Yaw.PID.In,CloudParam.Cloud_Gyro.Gyr_Z,Current);
		if(!CloudParam.Cloud_Gyro.Offline)
			CloudParam.Yaw.Target_Current=-Current1;
		else
			CloudParam.Yaw.Target_Current=-Current;
		
	}   
		else if(PID_Yaw==&CloudParam.Cloud_Gyro.Yaw_PID.Out)
	{
		Current1=pid_calc(&CloudParam.Cloud_Gyro.Yaw_PID.In,CloudParam.Cloud_Gyro.Gyr_Z,-Current);
		CloudParam.Yaw.Target_Current=-Current1;
	}

}

void Cloud_Current_Set(CAN_X_State CAN_X)
{
	static  CanSend_Type   SendData;

	
	SendData.CANx=CAN_X;
	SendData.SendCanTxMsg.StdId = 0x1FF;
	SendData.SendCanTxMsg.IDE = CAN_ID_STD;
	SendData.SendCanTxMsg.RTR = CAN_RTR_DATA;
	SendData.SendCanTxMsg.DLC = 0x08;
	SendData.SendCanTxMsg.Data[0] = CloudParam.Yaw.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[1] = CloudParam.Yaw.Target_Current;
	SendData.SendCanTxMsg.Data[2] = CloudParam.Pitch.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[3] = CloudParam.Pitch.Target_Current;
	SendData.SendCanTxMsg.Data[4] = M2006.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[5] = M2006.Target_Current;
	SendData.SendCanTxMsg.Data[6] = 0;
	SendData.SendCanTxMsg.Data[7] = 0;
	
	xQueueSend(Queue_CanSend, &SendData, 20);
}

