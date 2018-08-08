#include "Ctrl_Rammer.h"
#include "Ctrl_Frict.h"

const 	  int32_t 	Rammer_Max_Angle=((8192*REDUCTION_RATIO)-RAMMER_ERROR);//����תһȦ�������ۼƵ����ֵ
const     int32_t	St_Serve=(Rammer_Max_Angle/RAMMER_NUM+1);//תһ�������

uint8_t Hierogram_Shoot=0;
Shoot_Struct Shoot;
 /*
  * @brief ������������趨����������������������
  * @param None
  * @retval None
  */
void Rammer_Param_Set()//����
{
	if(M2006.Real_Current>4000)//��������4000��תһ��
	{
		M2006.Target_Angle=M2006.Target_Angle<=St_Serve?(Rammer_Max_Angle-2*St_Serve):M2006.Target_Angle-2*St_Serve;
	}
	else if(DBUS_ReceiveData.mouse.press_left||DBUS_ReceiveData.switch_left==2||Hierogram_Shoot==1)
	{
		if(DBUS_ReceiveData.mouse.press_right)//���ٸ�Ƶģʽ
		{
				switch(Judge_RobotState.robotLevel)//�����˵�ǰ�ȼ�
			{
				case 0:
				case 1://һ������ 90��ÿ����ȴ18
					FRICT_SPEED=650;//����
					Shoot.Freq=8;//��Ƶ��
					Shoot.Num=(120-Judge_RobotPowerHeatData.shooterHeat0)/14;
					break;
				case 2://��������180��ÿ����ȴ36
					FRICT_SPEED=750;//����
					Shoot.Freq=10;//��Ƶ��
					Shoot.Num=(240-Judge_RobotPowerHeatData.shooterHeat0)/18;
					break;
				case 3://��������360��ÿ����ȴ72
					FRICT_SPEED=800;//����
					Shoot.Freq=10;//��Ƶ��
					Shoot.Num=(480-Judge_RobotPowerHeatData.shooterHeat0)/20;
					break;
			}
		}
		else
		{
			switch(Judge_RobotState.robotLevel)//�����˵�ǰ�ȼ�
			{
				case 0:
				case 1://һ������ 90��ÿ����ȴ18
					FRICT_SPEED=1000;//����
					if(Judge_RobotRfidDetect.cardType&0x0100)//��ȴֵ�ӳ�
						Shoot.Freq=8;
					else
						Shoot.Freq=5;//��Ƶ��
					Shoot.Num=(120-Judge_RobotPowerHeatData.shooterHeat0)/22;
					break;
				case 2://��������180��ÿ����ȴ36
					FRICT_SPEED=1000;//����
					if(Judge_RobotRfidDetect.cardType&0x0100)
						Shoot.Freq=10;
					else
						Shoot.Freq=6;//��Ƶ��
					Shoot.Num=(240-Judge_RobotPowerHeatData.shooterHeat0)/22;
					break;
				case 3://��������360��ÿ����ȴ72
					FRICT_SPEED=1000;
					if(Judge_RobotRfidDetect.cardType&0x0100)
						Shoot.Freq=10;
					else
						Shoot.Freq=8;//��Ƶ��
					  Shoot.Num=(480-Judge_RobotPowerHeatData.shooterHeat0)/22;
					break;
			}
		} 
		if(Shoot.Num>2&&Shoot.Flag==1&&Mode!=SUPPLY)
		{
			Shoot.Flag=0;
			M2006.Target_Angle=M2006.Target_Angle>Rammer_Max_Angle?0:M2006.Target_Angle+St_Serve;
		}
		
	}
	else
	{
		Shoot.Time=0;
		Shoot.Flag=1;
	}
	
	M2006.Target_Angle=M2006.Target_Angle>Rammer_Max_Angle?0:M2006.Target_Angle;
	M2006_PID_Set();

}

 /*
  * @brief ����������ٱ�36��1��������ת������
  * @param None
  * @retval None
  */
void Rammer_Angle()
{
	static int16_t error=0;
	
	M2006.Now_Angle=M2006.Real_Angle;
	M2006.Error_Angle=M2006.Now_Angle-M2006.Last_Angle;//������α��������ݲ�ֵ
	
	if(M2006.Real_Speed>0)
	{
		if(M2006.Error_Angle<0)//����ٶȴ���0����ֵΪ������ʾ����㣬����8191
		{
			error=M2006.Error_Angle+8191;
			
		}
		else if(M2006.Error_Angle>=0)
		{
			error=M2006.Error_Angle;
			
		}
		if(M2006.Error_Angle>=0)//�Ѳ�ֵ�ۼ�
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
	
	M2006.Last_Angle=M2006.Now_Angle;//���ڵ�ֵ������һ�ε�ֵ
}

 /*
  * @brief �������PID����
  * @param None
  * @retval None
  */
void M2006_PID_Set()
{
	float Current1=0,Current2=0;
	
	Current1=pid_calc(&M2006.PID.Out,M2006.All_error_Angle,M2006.Target_Angle);
	Current2=pid_calc(&M2006.PID.In,M2006.Real_Speed,Current1);
	M2006.Target_Current=Current2;
}

