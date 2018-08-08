#include "control.h"
#include "Ctrl_Chassis.h"
#include "Ctrl_Cloud.h"
#include "Ctrl_Rammer.h"
#include "Ctrl_Frict.h"
#include "Driver_Display.h"
#include "User_can.h"
#include "Driver_Hierogram.h"
#include "Driver_Judge.h"
#include "Driver_SupCap.h"
#include "User.h"
#include "Driver_SupCap.h"
Current_Meter_Struct   Current_Meter;
 ChassisParam_Struct   ChassisParam;
   CloudParam_Struct   CloudParam;
     Motor2006_Param   M2006;
	
			uint8_t    Mode=COMMON;
		   uint16_t    First_Yaw_Angle,First_Pitch_Angle;//�ϵ���̨�����ʼλ��
              float    Mou_x,Mou_y;//����˲����ֵ

  volatile uint16_t    Cnt_1Hz, Cnt_10Hz, Cnt_20Hz, Cnt_50Hz;	
		    uint8_t	   Flag1_1Hz, Flag1_10Hz, Flag1_20Hz, Flag1_50Hz,
					   Flag2_1Hz, Flag2_10Hz, Flag2_20Hz, Flag2_50Hz,
					   Flag3_1Hz, Flag3_10Hz, Flag3_20Hz, Flag3_50Hz;

 /*
  * @brief ��ʱ���жϻص�����������������TIM7 10ms�ж��ﴦ��
  * @param *htim�����жϵĶ�ʱ���ṹ��ָ��
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	 
    if(htim == &htim7)//10ms�ж�һ��
    {   
		Frame_Rate_Statistics();//֡��ͳ��	
		Data_Display();//������λ����MiniBalance��OLED���ݷ�����ʾ
		Read_Cap_State();//����������
		Mode_Set();//ģʽ���� 
		if((DBUS_ReceiveData.switch_right==1||DBUS_ReceiveData.switch_right==3))		
		{		
			switch(Mode)
			{
/*****************************һ��ģʽ**********************************************/
				case    COMMON:						
							CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,2.2f,0,0);
							CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,6.5,0,0);						
							Hierogram_flag=0;
							Cloud_Param_Set();//��̨				
							Chassis_Param_Set(4000,4000);//���̣�����ٶ�4000
							Rammer_Param_Set();//����
							Frict_Param_Set();//Ħ����
							if((Judge_RobotGetBuff.buffType&0x01)&&DBUS_CheckPush(KEY_CTRL))
								Cap_Casual();//��Ѫ�㰴סCtrl��������������ݳ��
							else
								Cap_Mode();//��������0.5A���
							LED_Set(&Flag1_20Hz);//LED
							break;
/*****************************����ģʽ**********************************************/				
				case    SUPPLY:
							CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,2,0,0);
							CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,6.5,0,0);
							Cloud_Target_Angle(MEDIAN_PITCH-500,MEDIAN_YAW);
							Cloud_Param_Set();//��̨
							Chassis_Param_Set(800,800);//����
							Rammer_Param_Set();//����
							Frict_Param_Set();//Ħ����
							LED_Set(&Flag1_20Hz);	
							Cap_Casual();//����ģʽ��������������ݳ��
							break;				
/*****************************���ģʽ**********************************************/						
	 		case HIEROGRAM:
							Cloud_Target_Angle(DBUS_ReceiveData.mouse.y,DBUS_ReceiveData.mouse.x);
							CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,4,0,0);
                         	CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,5,0,2);
							Hierogram_Get();//��ȡС��������
							Cloud_Param_Set();//��̨
							Chassis_Param_Set(0,0);//����
						    Frict_Param_Set();//Ħ����
							Rammer_Param_Set();//����
							LED_Set(&Flag1_20Hz);
							break;
				default:
							break;
			}
			//д�����
			Set_moto_current(&hcan1,0x200, ChassisParam.LB.Target_Current, ChassisParam.RB.Target_Current, \
					ChassisParam.RF.Target_Current, ChassisParam.LF.Target_Current);
			Set_moto_current(&hcan1,0x1FF, CloudParam.Yaw.Target_Current, CloudParam.Pitch.Target_Current, \
					M2006.Target_Current, 0);
		}			
		else
		{
			Cap_Casual();//��������������ݳ��
			Frict_Param_Set();
			Set_moto_current(&hcan1,0x200, 0, 0, 0, 0);	
			Set_moto_current(&hcan1,0x1FF, 0, 0, 0, 0);	
		}
			
		
		if(Flag1_10Hz)//����������ʾ���ݣ��ٷ�Ҫ��10Hz
		{
			//������������е磬����6����ȫ��������ɫ��
			Flag1_10Hz=0;
			Transmit_Judge(22,ChassisParam.LB.Real_Speed,Shoot.Num,Judge_RobotPowerHeatData.chassisPowerBuffer,!CAP_OFF|(0xff*(CAP_READ_S))<<1);
		}
	}
    else if(htim == &htim6)//1ms�ж�һ��
    {
		if(++Cnt_1Hz>=1000)//1000ms
		{  
			Cnt_1Hz=0;
			Flag1_1Hz=1;
			Flag2_1Hz=1;
			Flag3_1Hz=1;//֡��ͳ��Ƶ��
		}	
		if(++Cnt_10Hz>=100)//100ms
		{
			Cnt_10Hz=0;
			Flag1_10Hz=1;//������������ˢ��Ƶ��
			Flag2_10Hz=1;//���Ե�OLED��ˢ��Ƶ��
			Flag3_10Hz=1;
		}
		
		if(++Cnt_20Hz>=50)//50ms
		{
			Cnt_20Hz=0;
			Flag1_20Hz=1;//����LED��˸Ƶ��
			Flag2_20Hz=1;
			Flag3_20Hz=1;
		}
		if(++Cnt_50Hz>=20)//20ms
		{
			Cnt_50Hz=0;
			Flag1_50Hz=1;
			Flag2_50Hz=1;
			Flag3_50Hz=1;
		}
		if(++Shoot.Time>=1000/Shoot.Freq)//����Ƶ��
		{
			Shoot.Time=0;
			Shoot.Flag=1;
		}
    }
}

 /*
  * @brief ���������������ģʽ����
  * @param None
  * @retval None
  */
void Mode_Set()
{
	if(DBUS_CheckPush(KEY_CTRL)&&DBUS_CheckPush(KEY_C))
	{
		 Mode=COMMON;
	}
	else if(DBUS_CheckPush(KEY_SHIFT)&&DBUS_CheckPush(KEY_X))
	{
		 Mode=HIEROGRAM;
	}
	else if(DBUS_CheckPush(KEY_CTRL)&&DBUS_CheckPush(KEY_X))
	{
		 Mode=SUPPLY;
	}
}

 /*
  * @brief ���豨�ķ���֡��ͳ�ƣ�Ƶ��1Hz
  * @param None
  * @retval None
  */
void Frame_Rate_Statistics()//֡��ͳ��1000msһ��
{
if(Flag3_1Hz)
{
	Flag3_1Hz=0;
	
	if(CloudParam.Gyro.FrameRate==0)
		CloudParam.Gyro.Offline=1;
	else
		CloudParam.Gyro.Offline=0;
	
	if(CloudParam.Pitch.FrameRate==0)
		CloudParam.Pitch.Offline=1;
	else
		CloudParam.Pitch.Offline=0;
	
	if(CloudParam.Yaw.FrameRate==0)
		CloudParam.Yaw.Offline=1;
	else
		CloudParam.Yaw.Offline=0;
	
	if(CloudParam.Cloud_Gyro.FrameRate==0)
	{
		CloudParam.Cloud_Gyro.Offline=1;
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);
	}
	else
	{
		CloudParam.Cloud_Gyro.Offline=0;
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);
	}
/******************************************************************/	
	if(ChassisParam.LB.FrameRate==0)
		ChassisParam.LB.Offline=1;
	else
		ChassisParam.LB.Offline=0;
	
	if(ChassisParam.LF.FrameRate==0)
		ChassisParam.LF.Offline=1;
	else
		ChassisParam.LF.Offline=0;
	
	if(ChassisParam.RB.FrameRate==0)
		ChassisParam.RB.Offline=1;
	else
		ChassisParam.RB.Offline=0;
	
	if(ChassisParam.RF.FrameRate==0)
		ChassisParam.RF.Offline=1;
	else
		ChassisParam.RF.Offline=0;
	
	if(ChassisParam.Chassis_Gyro.FrameRate==0)
		ChassisParam.Chassis_Gyro.Offline=1;
	else
		ChassisParam.Chassis_Gyro.Offline=0;
/******************************************************************/		
	if(M2006.FrameRate==0)
		M2006.Offline=1;
	else
		M2006.Offline=0;
	
	if(Judge.FrameRate==0)
	{
		Judge.Offline=1;
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);
	}
	else
	{
		Judge.Offline=0;
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET);
	}
	
	if(Remote.FrameRate==0)
		Remote.Offline=1;
	else
		Remote.Offline=0;
	
	if(PC_Data.FrameRate==0)
		PC_Data.Offline=1;
	else
		PC_Data.Offline=0;
	
	if(Cur_Meter.FrameRate==0)
	{
		Cur_Meter.Offline=1;
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_RESET);
	}
	else
	{
		Cur_Meter.Offline=0;
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_SET);
	}
/******************************************************************/	
	
	CloudParam.Gyro.FrameRate=0;
	CloudParam.Pitch.FrameRate=0;
	CloudParam.Yaw.FrameRate=0;
	CloudParam.Cloud_Gyro.FrameRate=0;
	
	ChassisParam.LB.FrameRate=0;
	ChassisParam.LF.FrameRate=0;
	ChassisParam.RB.FrameRate=0;
	ChassisParam.RF.FrameRate=0;
	ChassisParam.Chassis_Gyro.FrameRate=0;
	
	M2006.FrameRate=0;
	Judge.FrameRate=0;
	Remote.FrameRate=0;
	PC_Data.FrameRate=0;
	Cur_Meter.FrameRate=0;
}

}

 /*
  * @brief �˲�����������ƽ������
  * @param Val �˲�ǰ��*Value�˲���
  * @retval None
  */
void Filter(int16_t Val,float *Value,float Rate)
{

	*Value+=Rate*(Val-*Value);
	
}
