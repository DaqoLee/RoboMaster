#include "Driver_Display.h"
#include "control.h"
#include "Driver_Niming.h"
#include "Driver_Hierogram.h"
 /*
  * @brief LED��ͬ��ʽ��˸����ͬģʽ
  * @param *LED_HzΪ��˸Ƶ��
  * @retval None
  */
void LED_Set(uint8_t *LED_Hz)
{
	static uint8_t Flag=0;
	uint16_t Time=10*Mode*Mode;
	if(*LED_Hz)
	{
	    *LED_Hz=0;
		Flag=Flag>Time?0:Flag+1;
		if(Flag<Time/2)
		{	
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
		}
	}
	
}

 /*
  * @brief ������OLED����λ����������������ʾ
  * @param None
  * @retval None
  */
void Data_Display()
{
/*************OLED��ʾʱˢ��Ƶ�ʲ�Ҫ̫��***********************/
#if    0		
	
	if(Flag2_10Hz)//100msˢ��һ��
	{
		Flag2_10Hz=0;
//		Play_OLED_Chassis();
		OLED_ShowData_16(0*6,4,ChassisParam.Chassis_Gyro.Yaw,1);
		OLED_ShowData_16(10*6,4,ChassisParam.TargetABSAngle,1);
		
		OLED_ShowData_16(0*6,2,CloudParam.Cloud_Gyro.Yaw,1);
		OLED_ShowData_16(10*6,2,CloudParam.Cloud_Gyro.Target_Yaw,1);
		
		OLED_ShowData_16(0*6,6,ERror,1);
		OLED_ShowData_16(10*6,6,ERror+CloudParam.Cloud_Gyro.Target_Yaw,1);
//		OLED_ShowData_16(0*6,6,CloudParam.Pitch.Real_Angle,1);
//		OLED_ShowData_16(10*6,2,M2006.Real_Current,1);
//		OLED_ShowData_16(0*6,2,M2006.Last_Angle,1);
//		OLED_ShowData_16(0*6,0,M2006.Target_Current,1);
//		OLED_ShowData_16(10*6,0,error,1);
//		OLED_ShowData_16(10*6,4,M2006.Target_Angle,1);
//		OLED_ShowData_16(10*6,4,Judge_RobotPos.yaw,1);
				  
	}
#endif

/***********������λ����ʾʱUSART2����������Ϊ500000************/
	
#if    0
	if(Flag2_20Hz)
	{
		Flag2_20Hz=0; 
		Send_PC_Data(CloudParam.Cloud_Gyro.Yaw,CloudParam.Cloud_Gyro.Target_Yaw,0,0,\
		0,0);
	}		
#endif


/***********�����������ݸ��ֻ�ʱUSART2����������Ϊ9600***********/
#if	   0
	if(Flag2_10Hz)
	{
	    Flag2_10Hz=0;
//		HAL_UART_Transmit_DMA(&huart6,(uint8_t*)"OK\n",4);
//		printf("{B%d}$",(int)M2006.Position_PID.err[NOW]);
//		printf("Pitch %d\t%d   Yaw %d\t%d\n",(int)Pitch_Min,(int)Pitch_Max,(int)Yaw_Min,(int)Yaw_Max);
//		printf("{# %d }$",Rammer_Max_Angle);	
	}
				
#endif		
}
