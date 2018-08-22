#include "Ctrl_Cloud.h"

CloudParam_Struct   CloudParam;

const float	Pitch_Min=((8192.0f*(PITCH_RANGE/360.0f))/2+0.5f),Pitch_Max=(8192.0f-Pitch_Min+0.5f);//���ݽǶ����Ʒ�Χȷ��Pitch��С������е�Ƕ�
const float Yaw_Min=((8192.0f*(YAW_RANGE/360.0f))/2+0.5f),Yaw_Max=(8192.0f-Yaw_Min+0.5f);
const float	Gyro_Yaw_Min=YAW_GYRO_RANGE/2,Gyro_Yaw_Max=360-Gyro_Yaw_Min;


 /*
  * @brief ��̨�������������ز����Ĵ�����̨����ľ����Ǳ�������0~8191���������ǰ�װλ��Ӱ�죬Pitch����ʵ������Roll�滻
  * @param None
  * @retval None
  */

void Cloud_Param_Set(void)//��̨
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
		if(Game_Mode!=SUPPLY)//����ģʽʱ����Ƕ�����
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
	


	
	
		switch(Control_Mode)//�Ҽ����м�Ϊң��ģʽ��Ĭ����ģʽ1�����̸���
		{
/************************************************************************************************************************/
		case    Remote_1://ң��ģʽ(������̨)
					CloudParam.Pitch.Target_Angle=ABS(DBUS_ReceiveData.ch2)>20?CloudParam.Pitch.Target_Angle-DBUS_ReceiveData.ch2*0.02f:CloudParam.Pitch.Target_Angle;
					CloudParam.Yaw.Target_Angle=ABS(DBUS_ReceiveData.ch1)>20?CloudParam.Yaw.Target_Angle-DBUS_ReceiveData.ch1*0.02f:CloudParam.Yaw.Target_Angle;
					
					M6623_PID_Set(&CloudParam.Yaw.PID.Out,CloudParam.Yaw.Real_Angle ,CloudParam.Yaw.Target_Angle,\
					&CloudParam.Pitch.PID.Out,CloudParam.Pitch.Real_Angle,CloudParam.Pitch.Target_Angle);
					
					CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
					CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;

				break ;
				
/************************************************************************************************************************/				
		case    Remote_2://ң��ģʽ(��������̨)
			

		
				break ;
/************************************************************************************************************************/
		case    Keyboard://����ģʽ
	
				break ;
/************************************************************************************************************************/
		case    Ctrl_OFF://ʧ�ܿ���
					CloudParam.Pitch.Target_Current=0;
					CloudParam.Yaw.Target_Current=0;
		
				break ;
/************************************************************************************************************************/
		}
		Cloud_Current_Set(CAN_1);
}


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
		//CloudParam.Pitch.Target_Current=-Current1;
		CloudParam.Pitch.Target_Current=-Current;
	}
	else if(PID_Pitch==&CloudParam.Cloud_Gyro.Pitch_PID.Out)
	{
		Current1=pid_calc(&CloudParam.Cloud_Gyro.Pitch_PID.In,CloudParam.Cloud_Gyro.Gyr_Y,Current);
		CloudParam.Pitch.Target_Current=Current;
	}
//	else if(PID_Pitch==&Auto_Pitch_PID)
//	{
//		CloudParam.Pitch.Target_Current=-Current;
//	}
	
	//��̨Yaw��
	Current=pid_calc(PID_Yaw,Yaw_Real,Yaw_Target);
	if(PID_Yaw==&CloudParam.Yaw.PID.Out)
	{
		Current1=pid_calc(&CloudParam.Yaw.PID.In,CloudParam.Cloud_Gyro.Gyr_Z,Current);
		//CloudParam.Yaw.Target_Current=-Current1;
		CloudParam.Yaw.Target_Current=-Current;
		
	}   
		else if(PID_Yaw==&CloudParam.Cloud_Gyro.Yaw_PID.Out)
	{
		Current1=pid_calc(&CloudParam.Cloud_Gyro.Yaw_PID.In,CloudParam.Cloud_Gyro.Gyr_Z,-Current);
		CloudParam.Yaw.Target_Current=-Current1;
	}
//	else if(PID_Yaw==&Auto_Yaw_PID)
//	{
// 		CloudParam.Yaw.Target_Current=-Current;
//	}

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

