#include "Ctrl_Chassis.h"
#include "control.h"
#include "User_can.h"
#include "Driver_SupCap.h"
#include "Ctrl_Cloud.h"

float Buffer[4];
uint16_t MaxWheelSpeed=6600;

 /*
  * @brief �����ķ���˶�ģ��
  * @param Vx X�᷽����ٶȣ�Vy Y�᷽����ٶȣ�Omega �����ٶȣ�*Speed����ת��
  * @retval None
  */
void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed)
{
    float  Param, MaxSpeed;
    uint8_t index;
    
    Buffer[0] = -Vx - Vy + Omega;
    Buffer[1] = -Vx + Vy + Omega;
    Buffer[2] =  Vx + Vy + Omega;
    Buffer[3] =  Vx - Vy + Omega;
    
    //�޷�
    for(index = 0, MaxSpeed = 0; index < 4; index++)
    {
        if((Buffer[index] > 0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
        {
            MaxSpeed = (Buffer[index] > 0 ? Buffer[index] : -Buffer[index]);//���������ٶ�
        }
    }	
	
	if((MEDIAN_ROLL-CloudParam.Cloud_Gyro.Roll)-((CloudParam.Pitch.Real_Angle-(MEDIAN_PITCH-Pitch_Min-200))/22.75f)>12)
		MaxWheelSpeed=2000;//��̨�����ǽǶȱ仯����Pitch���������ת��ֵ������Ϊ�����£����ٶȽ�����
	else
		MaxWheelSpeed=6600;
	
    if(MaxWheelSpeed < MaxSpeed)
    {
        Param = (float)MaxWheelSpeed / MaxSpeed;
        Speed[0] = Buffer[0] * Param;
        Speed[1] = Buffer[1] * Param;
        Speed[2] = Buffer[2] * Param;
        Speed[3] = Buffer[3] * Param; 
    }
    else
    {
        Speed[0] = Buffer[0];
        Speed[1] = Buffer[1];
        Speed[2] = Buffer[2];
        Speed[3] = Buffer[3];
    }
}

 /*
  * @brief ���̹�������
  * @param None
  * @retval None
  */
void Power_Limit(float Cur_limit)
{
	static float Current;
    //�����ĸ�������ܵ���
	Current=ABS(ChassisParam.LF.Target_Current)+ABS(ChassisParam.LB.Target_Current)\
		   +ABS(ChassisParam.RB.Target_Current)+ABS(ChassisParam.RF.Target_Current);
	
	if(Current>Cur_limit)
	{//����Ŀ��ֵ�����������
		ChassisParam.LF.Target_Current = ChassisParam.LF.Target_Current/Current*Cur_limit;
		ChassisParam.LB.Target_Current = ChassisParam.LB.Target_Current/Current*Cur_limit;
		ChassisParam.RB.Target_Current = ChassisParam.RB.Target_Current/Current*Cur_limit;
		ChassisParam.RF.Target_Current = ChassisParam.RF.Target_Current/Current*Cur_limit;
	}
}

 /*
  * @brief ���̵��PID����
  * @param None
  * @retval None
  */
void M3508_PID_Set()
{
	float Current=0;
	
	//������ǰ��
	Current=pid_calc(&ChassisParam.LF.PID,ChassisParam.LF.Real_Speed,ChassisParam.LF.Target_Speed);
	ChassisParam.LF.Target_Current=Current;
	
	//���������
	Current=pid_calc(&ChassisParam.LB.PID,ChassisParam.LB.Real_Speed,ChassisParam.LB.Target_Speed);
	ChassisParam.LB.Target_Current=Current;
	
	//������ǰ��
	Current=pid_calc(&ChassisParam.RF.PID,ChassisParam.RF.Real_Speed,ChassisParam.RF.Target_Speed);
	ChassisParam.RF.Target_Current=Current;
	
	//�����Һ���
	Current=pid_calc(&ChassisParam.RB.PID,ChassisParam.RB.Real_Speed,ChassisParam.RB.Target_Speed);
	ChassisParam.RB.Target_Current=Current;
	
}

 /*
  * @brief ���̲�������
  * @param X��Y�������ٶ�
  * @retval None
  */
void Chassis_Param_Set(uint16_t Max_X,uint16_t Max_Y)//����
{
    static int16_t speed_x=0,speed_y=0;
	static int16_t Max_speed_x=0,Max_speed_y=0;
	static float Cur_limit;
	static int8_t Spin_Flag=1;
	int16_t WheelSpeed[4]; 
	if(!Remote.Offline)
	{
		if(DBUS_ReceiveData.switch_right==3)//ң��ģʽ
		{
			if(ABS(DBUS_ReceiveData.ch4)>20)//ң���м�ʱ������һ��Ϊ��
			{
				ChassisParam.TargetVY=10*DBUS_ReceiveData.ch4;
				if(!CloudParam.Yaw.Offline)
				{   //���̸��棬Yaw���������ʵ��ֵ���м�ֵ��
					Filter(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.2f);
					if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100||CloudParam.Cloud_Gyro.Offline)
						ChassisParam.TargetOmega=0;
					//��ֹ�ܶ�ʱ��̨����Ӱ�쳵�������е�Ƕ���200�Ķ�����Χ
				}
				else//���Yaw��������ֱ�ӿ��Ƶ���	
					ChassisParam.TargetOmega=ABS(DBUS_ReceiveData.ch1)>20?10*DBUS_ReceiveData.ch1:0;
			}
			else
				ChassisParam.TargetVY=0;
			
			if(ABS(DBUS_ReceiveData.ch3)>20)
			{
				ChassisParam.TargetVX=-10*DBUS_ReceiveData.ch3;
				if(!CloudParam.Yaw.Offline)
				{
					Filter(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.2f);
					if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100||CloudParam.Cloud_Gyro.Offline)
						ChassisParam.TargetOmega=0;
				}
				else
					ChassisParam.TargetOmega=ABS(DBUS_ReceiveData.ch1)>20?10*DBUS_ReceiveData.ch1:0;

			}
			else
				ChassisParam.TargetVX=0;
		
			if(ChassisParam.TargetVX==0&&ChassisParam.TargetVY==0)
			{
				if(!CloudParam.Yaw.Offline)
				{   //��ֹʱ��90�ȷ�Χ�ڲ�����
					Filter(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.06f);
					if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<1024||CloudParam.Cloud_Gyro.Offline)
						ChassisParam.TargetOmega=0;
				}
				else
					ChassisParam.TargetOmega=ABS(DBUS_ReceiveData.ch1)>20?10*DBUS_ReceiveData.ch1:0;		
			}
		}
		else if(DBUS_ReceiveData.switch_right==1)//����ģʽ
		{	
			if(Mode!=HIEROGRAM)
			{
				if((DBUS_CheckPush(KEY_D)||DBUS_CheckPush(KEY_A)||DBUS_CheckPush(KEY_S)||DBUS_CheckPush(KEY_W))&&DBUS_CheckPush(KEY_SHIFT))
				{	//��סShift����		
					if(CAP_READ_S)//�����������磬����ٶ����
					{
						Max_speed_x=6500;
						Max_speed_y=6500; 
					}
					else
					{
						Max_speed_x=5500;
						Max_speed_y=5500; 
					}
				}
				else if((DBUS_CheckPush(KEY_D)||DBUS_CheckPush(KEY_A)||DBUS_CheckPush(KEY_S)||DBUS_CheckPush(KEY_W))&&DBUS_CheckPush(KEY_CTRL))
				{   //��סCtrl����
					Max_speed_x=2000;
					Max_speed_y=2000; 
				}
				else
				{
					Max_speed_x=Max_speed_x<=Max_X?Max_X:Max_speed_x-20;
					Max_speed_y=Max_speed_y<=Max_Y?Max_Y:Max_speed_y-20;
				}	
				
				if(DBUS_CheckPush(KEY_W))
				{
					speed_y=speed_y<0?speed_y+100:speed_y;
					speed_y=speed_y>=Max_speed_y?Max_speed_y:speed_y+50;//��������
					if(!(DBUS_CheckPush(KEY_CTRL))&&!CloudParam.Cloud_Gyro.Offline&&Mode==COMMON)
						{
							if(DBUS_CheckPush(KEY_F))//��Ƥ��λ
							{		
								pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
								
								if(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW]>400)						
									Spin_Flag=1;
								else if(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW]<-400)
									Spin_Flag=-1;
								
								Filter(Spin_Flag*2000,&ChassisParam.TargetOmega,0.04f);
							}
							else//��ͨ����
							{
								ChassisParam.TargetOmega=pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
								if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100)
								ChassisParam.TargetOmega=0;
							}
						}
						else
							ChassisParam.TargetOmega=0;		
				}
				else if(DBUS_CheckPush(KEY_S))
				{
					speed_y=speed_y>0?speed_y-100:speed_y;
					speed_y=speed_y<=-Max_speed_y?-Max_speed_y:speed_y-50;
					if(!(DBUS_CheckPush(KEY_CTRL))&&!CloudParam.Cloud_Gyro.Offline&&Mode==COMMON)
						{
							ChassisParam.TargetOmega=pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
							if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100)
								ChassisParam.TargetOmega=0;	
						}
					else
						ChassisParam.TargetOmega=0;		
				}
				 else
				{
					speed_y=speed_y<0?speed_y+100:speed_y-100;//��������
					if(ABS(speed_y)<200)
					{
						speed_y=0;
					}
				}
				
				if(!DBUS_CheckPush(KEY_F))//û�а���F����������
				{
						if(!(DBUS_CheckPush(KEY_D)||DBUS_CheckPush(KEY_A)||DBUS_CheckPush(KEY_S)||DBUS_CheckPush(KEY_W)))
						{
							if(!(DBUS_CheckPush(KEY_CTRL))&&!CloudParam.Cloud_Gyro.Offline&&Mode==COMMON)
								{
									pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
									if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<1024)
										ChassisParam.TargetOmega=0;	
									else
										Filter(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.05f);
								}
									else
										ChassisParam.TargetOmega=0;	
						}
				}
				else if(!DBUS_CheckPush(KEY_W))//����F���Ҳ�ǰ��ʱŤ��
				{		
					pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
					
					if(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW]>600)				
						Spin_Flag=1;
					else if(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW]<-600)
						Spin_Flag=-1;
					
					if(Spin_Flag==1)
						ChassisParam.TargetOmega=ChassisParam.TargetOmega>3000?3000:ChassisParam.TargetOmega+200;
					else 
						ChassisParam.TargetOmega=ChassisParam.TargetOmega<-3000?-3000:ChassisParam.TargetOmega-200;
				}
				
				if(DBUS_CheckPush(KEY_D))
				{
					speed_x=speed_x>0?speed_x-100:speed_x;
					speed_x=speed_x<=-Max_speed_x?-Max_speed_x:speed_x-50;
					
					if(!(DBUS_CheckPush(KEY_CTRL))&&!CloudParam.Cloud_Gyro.Offline&&Mode==COMMON)
					{
						ChassisParam.TargetOmega=pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
						if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100)
							ChassisParam.TargetOmega=0;
					}
				}
				else if(DBUS_CheckPush(KEY_A))
				{
					speed_x=speed_x<0?speed_x+100:speed_x;
					speed_x=speed_x>=Max_speed_x?Max_speed_x:speed_x+50;
					
					if(!(DBUS_CheckPush(KEY_CTRL))&&!CloudParam.Cloud_Gyro.Offline&&Mode==COMMON)
					{
						ChassisParam.TargetOmega=pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
						if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100)
							ChassisParam.TargetOmega=0;
					}
				}
				else
				{
					speed_x=speed_x<0?speed_x+100:speed_x-100;
					if(ABS(speed_x)<200)
					{
						speed_x=0;
					}
	
				}
		if(Mode==SUPPLY)//����ģʽ�����棬Q��E����
		{
				if(DBUS_CheckPush(KEY_Q))
					 ChassisParam.TargetOmega=-500;//����ʱ����
				
				else if(DBUS_CheckPush(KEY_E))
				
					ChassisParam.TargetOmega=500;
				
				else
					ChassisParam.TargetOmega=0;
		 }
				Filter(speed_x,&ChassisParam.TargetVX,0.1f);//����ƽ������
				Filter(speed_y,&ChassisParam.TargetVY,0.1f);

		}
		else//�ֶ�������ٶ�Ϊ��
		{
			ChassisParam.TargetVX=0;
			ChassisParam.TargetVY=0;
		}
	 }
	}
   else	//ң�����ߣ��ٶ�ȫ��Ϊ��
	{
		ChassisParam.TargetVX=0;
		ChassisParam.TargetVY=0;
		ChassisParam.TargetOmega=0;		
	}
	
	MecanumCalculate(ChassisParam.TargetVX,ChassisParam.TargetVY,ChassisParam.TargetOmega,WheelSpeed);
	//���ֽ���
	ChassisParam.LF.Target_Speed = WheelSpeed[0];
    ChassisParam.LB.Target_Speed = WheelSpeed[1];
    ChassisParam.RB.Target_Speed = WheelSpeed[2];
    ChassisParam.RF.Target_Speed = WheelSpeed[3];
	
	M3508_PID_Set();
	
	if(Judge_Power_Limit==1||Meter_Power_Limit==1)
	{//���幦�ʵ���50
		
		if(!Judge.Offline||!Cur_Meter.FrameRate)//�����ƻ����ϵͳ�������������й�������
			Cur_limit=100000.0f/Judge_RobotPowerHeatData.chassisVolt;
		else
			Cur_limit=3000.0f;
		 Power_Limit(Cur_limit);
	}
	else if(Judge_Power_Limit==2||Meter_Power_Limit==2)
	{//���幦�ʵ���30	
		if(!Judge.Offline||!Cur_Meter.FrameRate)
			Cur_limit=80000.0f/Judge_RobotPowerHeatData.chassisVolt;
		else
			Cur_limit=3000.0f;
		 Power_Limit(Cur_limit);
	}
	
}

