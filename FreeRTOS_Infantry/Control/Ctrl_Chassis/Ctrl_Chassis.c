#include "Ctrl_Chassis.h"
#include "Driver_DBUS.h"
#include "Ctrl_Cloud.h"

ChassisParam_Struct   ChassisParam;

 /*
  * @brief ���̲�������
  * @param X��Y�������ٶ�
  * @retval None
  */
void Chassis_Param_Set(uint16_t Max_X,uint16_t Max_Y)//����
{
	if(DBUS_ReceiveData.switch_right==3)//ң��ģʽ
	{
		if(ABS(DBUS_ReceiveData.ch4)>20)//ң���м�ʱ������һ��Ϊ��
		{
			ChassisParam.TargetVY=10*DBUS_ReceiveData.ch4;
			if(!CloudParam.Yaw.Offline)
			{   //���̸��棬Yaw���������ʵ��ֵ���м�ֵ��
				Filters(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.2f);
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
				Filters(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.2f);
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
				Filters(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.06f);
				if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<1024||CloudParam.Cloud_Gyro.Offline)
					ChassisParam.TargetOmega=0;
			}
			else
				ChassisParam.TargetOmega=ABS(DBUS_ReceiveData.ch1)>20?10*DBUS_ReceiveData.ch1:0;		
		}
	}
}
 /*
  * @brief �����ķ���˶�ģ��
  * @param Vx X�᷽����ٶȣ�Vy Y�᷽����ٶȣ�Omega �����ٶȣ�*Speed����ת��
  * @retval None
  */
void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed)
{
	static float Buffer[4];
	static uint16_t MaxWheelSpeed=6600;
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
	
//	if((MEDIAN_ROLL-CloudParam.Cloud_Gyro.Roll)-((CloudParam.Pitch.Real_Angle-(MEDIAN_PITCH-Pitch_Min-200))/22.75f)>12)
//		MaxWheelSpeed=2000;//��̨�����ǽǶȱ仯����Pitch���������ת��ֵ������Ϊ�����£����ٶȽ�����
//	else
//		MaxWheelSpeed=6600;
	
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
	//������ǰ��
	ChassisParam.LF.Target_Current=pid_calc(&ChassisParam.LF.PID,ChassisParam.LF.Real_Speed,ChassisParam.LF.Target_Speed);
	
	//���������
	ChassisParam.LB.Target_Current=pid_calc(&ChassisParam.LB.PID,ChassisParam.LB.Real_Speed,ChassisParam.LB.Target_Speed);
	
	//������ǰ��
	ChassisParam.RF.Target_Current=pid_calc(&ChassisParam.RF.PID,ChassisParam.RF.Real_Speed,ChassisParam.RF.Target_Speed);
	
	//�����Һ���
	ChassisParam.RB.Target_Current=pid_calc(&ChassisParam.RB.PID,ChassisParam.RB.Real_Speed,ChassisParam.RB.Target_Speed);
}

void Set_moto_current(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
{

	hcan->pTxMsg->StdId = ID;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = Current1 >> 8;
	hcan->pTxMsg->Data[1] = Current1;
	hcan->pTxMsg->Data[2] = Current2 >> 8;
	hcan->pTxMsg->Data[3] = Current2;
	hcan->pTxMsg->Data[4] = Current3 >> 8;
	hcan->pTxMsg->Data[5] = Current3 ;
	hcan->pTxMsg->Data[6] = Current4 >> 8;
	hcan->pTxMsg->Data[7] = Current4;
	
	
	xQueueSend(Queue_CanSend, hcan, 20);
	
}

