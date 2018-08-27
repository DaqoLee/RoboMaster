#include "Driver_Gyro.h"

uint8_t Gyro_Init_flag=0;//陀螺仪初始化完成标志
uint8_t Cloud_Gyro_Data[20],Chassis_Gyro_Data[20];
int16_t ERror;

 /*
  * @brief 云台陀螺仪报文解析
  * @param None
  * @retval None
  */
void Analysis_Cloud_Gyro()
{
	static uint8_t i=0,sum;
	
		for(sum=0,i=0;i<(Cloud_Gyro_Data[3]+4);i++)
			sum+=Cloud_Gyro_Data[i];
		if(sum==Cloud_Gyro_Data[i])//校验和判断
		{
			
			CloudParam.Cloud_Gyro.Gyr_X=((Cloud_Gyro_Data[4]<<8)|Cloud_Gyro_Data[5]);
			CloudParam.Cloud_Gyro.Gyr_Y=((Cloud_Gyro_Data[6]<<8)|Cloud_Gyro_Data[7]);
			CloudParam.Cloud_Gyro.Gyr_Z=((Cloud_Gyro_Data[8]<<8)|Cloud_Gyro_Data[9]);
	
			CloudParam.Cloud_Gyro.Yaw=(uint16_t)((Cloud_Gyro_Data[10]<<8)|Cloud_Gyro_Data[11])/100.0f;
			CloudParam.Cloud_Gyro.Roll=(int16_t)((Cloud_Gyro_Data[12]<<8)|Cloud_Gyro_Data[13])/100.0f;
			CloudParam.Cloud_Gyro.Pitch=(int16_t)((Cloud_Gyro_Data[14]<<8)|Cloud_Gyro_Data[15])/100.0f;
			
			CloudParam.Cloud_Gyro.FrameRate++;
		}
}

 /*
  * @brief 底盘陀螺仪报文解析
  * @param None
  * @retval None
  */
void Analysis_Chassis_Gyro()
{
	static uint8_t i=0,sum;
	
		for(sum=0,i=0;i<(Chassis_Gyro_Data[3]+4);i++)
			sum+=Chassis_Gyro_Data[i];
		if(sum==Chassis_Gyro_Data[i])//校验和判断
		{
			
			ChassisParam.Chassis_Gyro.Gyr_X=((Chassis_Gyro_Data[4]<<8)|Chassis_Gyro_Data[5]);
			ChassisParam.Chassis_Gyro.Gyr_Y=((Chassis_Gyro_Data[6]<<8)|Chassis_Gyro_Data[7]);
			ChassisParam.Chassis_Gyro.Gyr_Z=((Chassis_Gyro_Data[8]<<8)|Chassis_Gyro_Data[9]);
	
			ChassisParam.Chassis_Gyro.Yaw=(uint16_t)((Chassis_Gyro_Data[10]<<8)|Chassis_Gyro_Data[11])/100.0f;
			ChassisParam.Chassis_Gyro.Roll=(int16_t)((Chassis_Gyro_Data[12]<<8)|Chassis_Gyro_Data[13])/100.0f;
			ChassisParam.Chassis_Gyro.Pitch=(int16_t)((Chassis_Gyro_Data[14]<<8)|Chassis_Gyro_Data[15])/100.0f;
			
			ChassisParam.Chassis_Gyro.FrameRate++;
		}
}

