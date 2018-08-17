#include "Ctrl_Rammer.h"

const 	  int32_t 	Rammer_Max_Angle=((8192*REDUCTION_RATIO)-RAMMER_ERROR);//拨盘转一圈编码器累计的最大值
const     int32_t	St_Serve=(Rammer_Max_Angle/RAMMER_NUM+1);//转一格的线数


Motor2006_Param   M2006;

void M2006_PID_Set(void)
{
	float Current1=0,Current2=0;
	
	Current1=pid_calc(&M2006.PID.Out,M2006.All_error_Angle,M2006.Target_Angle);
	Current2=pid_calc(&M2006.PID.In,M2006.Real_Speed,Current1);
	M2006.Target_Current=Current2;
}

/*
  * @brief 拨弹电机减速比36：1，编码器转换函数
  * @param None
  * @retval None
  */
void Rammer_Angle(void)
{
	static int16_t error=0;
	
	M2006.Now_Angle=M2006.Real_Angle;
	M2006.Error_Angle=M2006.Now_Angle-M2006.Last_Angle;//获得两次编码器数据差值
	
	if(M2006.Real_Speed>0)
	{
		if(M2006.Error_Angle<0)//如果速度大于0，差值为负数表示过零点，加上8191
		{
			error=M2006.Error_Angle+8191;
			
		}
		else if(M2006.Error_Angle>=0)
		{
			error=M2006.Error_Angle;
			
		}
		if(M2006.Error_Angle>=0)//把差值累加
		M2006.All_error_Angle=M2006.All_error_Angle>Rammer_Max_Angle?M2006.All_error_Angle-Rammer_Max_Angle:M2006.All_error_Angle+error;
	}
	else if(M2006.Real_Speed<0)
	{
		if(M2006.Error_Angle>0)
		{
			error=M2006.Error_Angle-8191;
		
		}
		else if(M2006.Error_Angle<=0)
		{
			error=M2006.Error_Angle;
			
		}
		if(M2006.Error_Angle<=0)
			
		M2006.All_error_Angle=M2006.All_error_Angle<0?M2006.All_error_Angle+Rammer_Max_Angle:M2006.All_error_Angle+error;
	}
	
	M2006.Last_Angle=M2006.Now_Angle;//现在的值赋给上一次的值
}