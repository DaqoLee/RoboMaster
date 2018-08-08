#include "Ctrl_Cloud.h"
#include "Driver_Hierogram.h"
const	    float	Pitch_Min=((8192.0f*(PITCH_RANGE/360.0f))/2+0.5f),Pitch_Max=(8192.0f-Pitch_Min+0.5f);//���ݽǶ����Ʒ�Χȷ��Pitch��С������е�Ƕ�
const       float   Yaw_Min=((8192.0f*(YAW_RANGE/360.0f))/2+0.5f),Yaw_Max=(8192.0f-Yaw_Min+0.5f);
const       float	Gyro_Yaw_Min=YAW_GYRO_RANGE/2,Gyro_Yaw_Max=360-Gyro_Yaw_Min;

 /*
  * @brief ��̨���PID����
  * @param Yaw����Pitch���PID�ṹ�壬ʵ��ֵ��Ŀ��ֵ��
  * @retval None
  */
void M6623_PID_Set(pid_t* PID_Yaw, float Yaw_Real,float Yaw_Target,pid_t* PID_Pitch, float Pitch_Real,float Pitch_Target)
{
	float Current=0,Current1=0;
	//��̨Pitch��
	Current=pid_calc(PID_Pitch,Pitch_Real,Pitch_Target);//�⻷PID����
	if(PID_Pitch==&CloudParam.Pitch.PID.Out)//�ж��⻷����
	{
		Current1=pid_calc(&CloudParam.Pitch.PID.In,CloudParam.Cloud_Gyro.Gyr_Y,Current);//�ڻ�PID����
		CloudParam.Pitch.Target_Current=-Current1;
	}
	else if(PID_Pitch==&CloudParam.Cloud_Gyro.Pitch_PID.Out)
	{
		Current1=pid_calc(&CloudParam.Cloud_Gyro.Pitch_PID.In,CloudParam.Cloud_Gyro.Gyr_Y,Current);
		CloudParam.Pitch.Target_Current=Current;
	}
	else if(PID_Pitch==&Auto_Pitch_PID)
	{
		CloudParam.Pitch.Target_Current=-Current;
	}
	
	//��̨Yaw��
	Current=pid_calc(PID_Yaw,Yaw_Real,Yaw_Target);
	if(PID_Yaw==&CloudParam.Yaw.PID.Out)
	{
		Current1=pid_calc(&CloudParam.Yaw.PID.In,CloudParam.Cloud_Gyro.Gyr_Z,Current);
		CloudParam.Yaw.Target_Current=-Current1;
		
	}   
		else if(PID_Yaw==&CloudParam.Cloud_Gyro.Yaw_PID.Out)
	{
		Current1=pid_calc(&CloudParam.Cloud_Gyro.Yaw_PID.In,CloudParam.Cloud_Gyro.Gyr_Z,-Current);
		CloudParam.Yaw.Target_Current=-Current1;
	}
	else if(PID_Yaw==&Auto_Yaw_PID)
	{
 		CloudParam.Yaw.Target_Current=-Current;
	}

}

 /*
  * @brief ��̨�������������ز����Ĵ�����̨����ľ����Ǳ�������0~8191���������ǰ�װλ��Ӱ�죬Pitch����ʵ������Roll�滻
  * @param None
  * @retval None
  */

void Cloud_Param_Set()//��̨
{	

/**************************************��Ŀ��ֵ������0~8191**************************************/	
	CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Target_Angle>8191?CloudParam.Pitch.Target_Angle-8191:CloudParam.Pitch.Target_Angle;
	CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Target_Angle<0?CloudParam.Pitch.Target_Angle+8191:CloudParam.Pitch.Target_Angle;
	
	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle>8191?CloudParam.Yaw.Target_Angle-8191:CloudParam.Yaw.Target_Angle;
	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle<0?CloudParam.Yaw.Target_Angle+8191:CloudParam.Yaw.Target_Angle;
	
	CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>360?CloudParam.Cloud_Gyro.Target_Yaw-360:CloudParam.Cloud_Gyro.Target_Yaw;
	CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<0?CloudParam.Cloud_Gyro.Target_Yaw+360:CloudParam.Cloud_Gyro.Target_Yaw;
	
	CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Target_Roll>10?10:CloudParam.Cloud_Gyro.Target_Roll;
	CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Target_Roll<-15?-15:CloudParam.Cloud_Gyro.Target_Roll;
/***********************************Yaw��������180��͹��㴦��************************************/	
	if(MEDIAN_YAW>Yaw_Min&&MEDIAN_YAW<Yaw_Max)//�м�ֵ�����ֵ����Сֵ֮�䣬���ᾭ�����
	{
		
	  	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle>MEDIAN_YAW+Yaw_Min?MEDIAN_YAW+Yaw_Min:CloudParam.Yaw.Target_Angle;
	    CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Target_Angle<MEDIAN_YAW-Yaw_Min?MEDIAN_YAW-Yaw_Min:CloudParam.Yaw.Target_Angle;
		
	}
	
	else if(MEDIAN_YAW<Yaw_Min)//�м�ֵС����Сֵ�����㴦��
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
	
/***********************************Pitch��������60��͹��㴦��************************************/	
	if(MEDIAN_PITCH>Pitch_Min&&MEDIAN_PITCH<Pitch_Max)
	{
		if(Mode!=SUPPLY)//����ģʽʱ����Ƕ�����
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
	
#if  INFANTRY==4	
//	if(ChassisParam.Chassis_Gyro.Yaw-ERror>Gyro_Yaw_Min&&ChassisParam.Chassis_Gyro.Yaw-ERror<Gyro_Yaw_Max)
//	{
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>359?2:CloudParam.Cloud_Gyro.Target_Yaw;
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<1?358:CloudParam.Cloud_Gyro.Target_Yaw;
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw-(ChassisParam.Chassis_Gyro.Yaw-ERror)>Gyro_Yaw_Min?\
//		(ChassisParam.Chassis_Gyro.Yaw-ERror)+Gyro_Yaw_Min:CloudParam.Cloud_Gyro.Target_Yaw;
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw-(ChassisParam.Chassis_Gyro.Yaw-ERror)<-Gyro_Yaw_Min?\
//		(ChassisParam.Chassis_Gyro.Yaw-ERror)-Gyro_Yaw_Min:CloudParam.Cloud_Gyro.Target_Yaw;
//	}
//	else if((ChassisParam.Chassis_Gyro.Yaw-ERror)<=Gyro_Yaw_Min)
//	{
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw-(ChassisParam.Chassis_Gyro.Yaw-ERror)>Gyro_Yaw_Min\
//		&&CloudParam.Cloud_Gyro.Target_Yaw<(ChassisParam.Chassis_Gyro.Yaw-ERror)+180?(ChassisParam.Chassis_Gyro.Yaw-ERror)+Gyro_Yaw_Min:CloudParam.Cloud_Gyro.Target_Yaw;
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<(ChassisParam.Chassis_Gyro.Yaw-ERror)+Gyro_Yaw_Max\
//		&&CloudParam.Cloud_Gyro.Target_Yaw>(ChassisParam.Chassis_Gyro.Yaw-ERror)+180?(ChassisParam.Chassis_Gyro.Yaw-ERror)+Gyro_Yaw_Max:CloudParam.Cloud_Gyro.Target_Yaw;

//	}
//	
//	else if((ChassisParam.Chassis_Gyro.Yaw-ERror)>=Gyro_Yaw_Max)
//	{
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<(ChassisParam.Chassis_Gyro.Yaw-ERror)-Gyro_Yaw_Min\
//		&&CloudParam.Cloud_Gyro.Target_Yaw>(ChassisParam.Chassis_Gyro.Yaw-ERror)-180?(ChassisParam.Chassis_Gyro.Yaw-ERror)-Gyro_Yaw_Min:CloudParam.Cloud_Gyro.Target_Yaw;
//		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>(ChassisParam.Chassis_Gyro.Yaw-ERror)-Gyro_Yaw_Max\
//		&&CloudParam.Cloud_Gyro.Target_Yaw<(ChassisParam.Chassis_Gyro.Yaw-ERror)-180?(ChassisParam.Chassis_Gyro.Yaw-ERror)-Gyro_Yaw_Max:CloudParam.Cloud_Gyro.Target_Yaw;

//	}
#endif	
#if  	1	
if(DBUS_ReceiveData.switch_right==1&&Mode==COMMON)
{
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
//			Filter(0.04f*DBUS_ReceiveData.mouse.x,&Mou_x,0.5f);//�˲�����
//	    	Filter(0.8f*DBUS_ReceiveData.mouse.y,&Mou_y,0.5f);//�˲�����
//			CloudParam.Cloud_Gyro.Target_Yaw+=Mou_x;
//			CloudParam.Pitch.Target_Angle-=Mou_y;

			//��������ٶ�
			DBUS_ReceiveData.mouse.x=DBUS_ReceiveData.mouse.x>200?200:DBUS_ReceiveData.mouse.x;
			DBUS_ReceiveData.mouse.x=DBUS_ReceiveData.mouse.x<-200?-200:DBUS_ReceiveData.mouse.x;
			
			DBUS_ReceiveData.mouse.y=DBUS_ReceiveData.mouse.y>200?200:DBUS_ReceiveData.mouse.y;
			DBUS_ReceiveData.mouse.y=DBUS_ReceiveData.mouse.y<-200?-200:DBUS_ReceiveData.mouse.y;
			
			CloudParam.Cloud_Gyro.Target_Yaw+=0.04f*DBUS_ReceiveData.mouse.x;
			CloudParam.Cloud_Gyro.Target_Roll+=0.02f*DBUS_ReceiveData.mouse.y;
			CloudParam.Pitch.Target_Angle-=0.8f*DBUS_ReceiveData.mouse.y;
			
		}
#if  INFANTRY==4
			if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
				||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
		{
				CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
		}
#elif INFANTRY==1

			if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
				||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
		{
				CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
		}
#endif
		
		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>360?CloudParam.Cloud_Gyro.Target_Yaw-360:CloudParam.Cloud_Gyro.Target_Yaw;
	    CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<0?CloudParam.Cloud_Gyro.Target_Yaw+360:CloudParam.Cloud_Gyro.Target_Yaw;
		
		M6623_PID_Set(&CloudParam.Cloud_Gyro.Yaw_PID.Out,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
		&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
		CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
		
//		M6623_PID_Set(&CloudParam.Cloud_Gyro.Yaw_PID.Out,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
//		&CloudParam.Cloud_Gyro.Pitch_PID.Out,CloudParam.Cloud_Gyro.Roll,CloudParam.Cloud_Gyro.Target_Roll);
//		CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
//		CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Real_Angle;
	}	
	else
	{
		CloudParam.Pitch.Target_Angle-=0.5f*Mou_y;
		CloudParam.Yaw.Target_Angle-=0.5f*Mou_x;
		//��̨PID����
		M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,CloudParam.Yaw.Target_Angle,\
		&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
		
		CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;//��ʵֵ��ΪĿ��ֵ���л�ʱ��֤λ�ò���
		CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
	}
}
else
#endif
	
{
	if(!CloudParam.Cloud_Gyro.Offline)
	{
		CloudParam.Cloud_Gyro.Target_Yaw=ABS(DBUS_ReceiveData.ch1)>20?CloudParam.Cloud_Gyro.Target_Yaw+DBUS_ReceiveData.ch1*0.005:CloudParam.Cloud_Gyro.Target_Yaw;
		CloudParam.Cloud_Gyro.Target_Roll=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Cloud_Gyro.Target_Roll-DBUS_ReceiveData.ch2*0.005:CloudParam.Cloud_Gyro.Target_Roll;
	    CloudParam.Pitch.Target_Angle=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Pitch.Target_Angle+DBUS_ReceiveData.ch2*0.05f:CloudParam.Pitch.Target_Angle;
     	
//			CloudParam.Cloud_Gyro.Target_Yaw+=DBUS_ReceiveData.ch1*0.002;
//			CloudParam.Cloud_Gyro.Target_Roll+=DBUS_ReceiveData.ch2*0.002;
		
#if  INFANTRY==4
	
			if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
				||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
		{
				CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
		}

#elif INFANTRY==1


		if((CloudParam.Yaw.Real_Angle<MEDIAN_YAW-Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]>0)\
			||(CloudParam.Yaw.Real_Angle>MEDIAN_YAW+Yaw_Min&&CloudParam.Cloud_Gyro.Yaw_PID.Out.err[NOW]<0))
		{
			CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
		}
#endif
		if(Mode==COMMON)
		{
		  	CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw>360?CloudParam.Cloud_Gyro.Target_Yaw-360:CloudParam.Cloud_Gyro.Target_Yaw;
	        CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Target_Yaw<0?CloudParam.Cloud_Gyro.Target_Yaw+360:CloudParam.Cloud_Gyro.Target_Yaw;
			M6623_PID_Set(&CloudParam.Cloud_Gyro.Yaw_PID.Out,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
			&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
			
//			M6623_PID_Set(&CloudParam.Cloud_Gyro.Position_Yaw_PID,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
//			&CloudParam.Cloud_Gyro.Position_Pitch_PID,CloudParam.Cloud_Gyro.Roll,CloudParam.Cloud_Gyro.Target_Roll);
//			CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
//			CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Real_Angle;
//	    	M6623_PID_Set(&CloudParam.Cloud_Gyro.Yaw_PID.Out,CloudParam.Cloud_Gyro.Yaw ,CloudParam.Cloud_Gyro.Target_Yaw,\
//		    &CloudParam.Cloud_Gyro.Pitch_PID.Out,CloudParam.Cloud_Gyro.Roll,CloudParam.Cloud_Gyro.Target_Roll);
//		    CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
//		    CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Real_Angle;
		}
		else 
		{
////			if(!PC_Data.Offline)
////			{	
//				//Filter(Hierogram_X,&Target_Auto_Yaw,0.1f);
//				//Filter(Hierogram_Y,&Target_Auto_Pitch,0.1f);
//				M6623_PID_Set(&Auto_Yaw_PID,CloudParam.Yaw.Real_Angle,Target_Auto_Yaw,&Auto_Pitch_PID,CloudParam.Pitch.Real_Angle,Target_Auto_Pitch);
//			//	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
//			
//			//	M6623_PID_Set(&Auto_Yaw_PID,Hierogram_X,400,&Auto_Pitch_PID,Hierogram_Y,300);
//			//	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
////	  		    CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Real_Angle;
////			}
////			else
			M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,CloudParam.Yaw.Target_Angle,\
			&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
			
			CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
			CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
		
	}
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
	
}

}

 /*
  * @brief ��̨��Yaw��Pitch���Ŀ��ֵ�趨����
  * @param Pitch��Yaw Ŀ��ֵ
  * @retval None
  */
void Cloud_Target_Angle(int16_t Pitch,int16_t Yaw)
{
	if(Pitch==DBUS_ReceiveData.ch2&&Yaw==DBUS_ReceiveData.ch1)
	{
		CloudParam.Pitch.Target_Angle+=Pitch/30;
		CloudParam.Yaw.Target_Angle-=Yaw/30;
	}
	else if(Pitch==DBUS_ReceiveData.mouse.y&&Yaw==DBUS_ReceiveData.mouse.x)
	{
		CloudParam.Pitch.Target_Angle-=0.5*Pitch;
		CloudParam.Yaw.Target_Angle-=0.5*Yaw;
	}
	else
	{
		CloudParam.Pitch.Target_Angle=Pitch;
		CloudParam.Yaw.Target_Angle=Yaw;
	}
}

void Read_M6623_First_Angle()//��ȡYaw���ʼλ��
{
    CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
	CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Real_Angle;

}


