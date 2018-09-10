#include "Ctrl_Chassis.h"

ChassisParam_Struct   ChassisParam;
 /*
  * @brief 底盘参数设置
  * @param X与Y轴的最大速度
  * @retval None
  */
void Chassis_Param_Set(void)//底盘
{
	int16_t WheelSpeed[4]; 
	static int16_t Target_VX=0,Target_VY=0;
	static int16_t Speed_X=0,Speed_Y=0;
	static int8_t Spin_Flag=1;
	switch(Control_Mode)//右键在中间为遥控模式，默认是模式1，底盘跟随
	{
/************************************************************************************************************************/
		case    Remote_1://遥控模式(跟随云台)
			
				if(ABS(DBUS_ReceiveData.ch4)>20)//遥控中间时有误差，不一定为零
				{
					ChassisParam.TargetVY=10*DBUS_ReceiveData.ch4;
					if(!CloudParam.Yaw.Offline)
					{   //底盘跟随，Yaw轴编码器的实际值往中间值靠
						Filters(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.2f);
						if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100||CloudParam.Cloud_Gyro.Offline)
							ChassisParam.TargetOmega=0;
						//防止跑动时云台抖动影响车身，允许机械角度有200的抖动范围
					}
					else//如果Yaw轴电机离线直接控制底盘	
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
						if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<150||CloudParam.Cloud_Gyro.Offline)
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
					{   //静止时，90度范围内不跟随
						Filters(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.06f);
						if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<600||CloudParam.Cloud_Gyro.Offline)
							ChassisParam.TargetOmega=0;
					}
					else
 						ChassisParam.TargetOmega=ABS(DBUS_ReceiveData.ch1)>20?10*DBUS_ReceiveData.ch1:0;		
				}
				
				MecanumCalculate(ChassisParam.TargetVX,ChassisParam.TargetVY,ChassisParam.TargetOmega,WheelSpeed);
				//麦轮解算
				ChassisParam.LF.Target_Speed = WheelSpeed[0];
				ChassisParam.LB.Target_Speed = WheelSpeed[1];
				ChassisParam.RB.Target_Speed = WheelSpeed[2];
				ChassisParam.RF.Target_Speed = WheelSpeed[3];
				M3508_PID_Set();
				break ;
				
/************************************************************************************************************************/				
		case    Remote_2://遥控模式(不跟随云台)
			
				ChassisParam.TargetVY=ABS(DBUS_ReceiveData.ch4)>20?10*DBUS_ReceiveData.ch4:0;
				ChassisParam.TargetVX=ABS(DBUS_ReceiveData.ch3)>20?-10*DBUS_ReceiveData.ch3:0;
				ChassisParam.TargetOmega=ABS(DBUS_ReceiveData.ch1)>20?10*DBUS_ReceiveData.ch1:0;	
		
				MecanumCalculate(ChassisParam.TargetVX,ChassisParam.TargetVY,ChassisParam.TargetOmega,WheelSpeed);
				//麦轮解算
				ChassisParam.LF.Target_Speed = WheelSpeed[0];
				ChassisParam.LB.Target_Speed = WheelSpeed[1];
				ChassisParam.RB.Target_Speed = WheelSpeed[2];
				ChassisParam.RF.Target_Speed = WheelSpeed[3];
				M3508_PID_Set();
		
			break ;
/************************************************************************************************************************/
		case    Keyboard://键鼠模式
			
				Key_Combination();//组合按键
				switch(Game_Mode)
				{
/************************************************************************************************************************/
					case    COMMON://一般
								switch(Speed_Mode)
								{
									case    UP://加速
											Target_VX=SPEED_UP;
											Target_VY=SPEED_UP;
											break ;
									case    Down://减速
											Target_VX=SPEED_DOWN;
											Target_VY=SPEED_DOWN;
											break ;
									case    Normal://正常
											Target_VX=SPEED_NORMAL;
											Target_VY=SPEED_NORMAL;
											break ;
									default:
											Target_VX=SPEED_NORMAL;
											Target_VY=SPEED_NORMAL;
											break;
								}
								if(DBUS_CheckPush(KEY_W))
								{
									Speed_Y=Speed_Y<0?Speed_Y+2*SPEED_BUFFER:Speed_Y;
									Speed_Y=Speed_Y>=Target_VY?Target_VY:Speed_Y+SPEED_BUFFER;
									
									if(!CloudParam.Cloud_Gyro.Offline)
										{
											if(DBUS_CheckPush(KEY_F))//蛇皮走位
											{		
												pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
												
												if(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW]>400)						
													Spin_Flag=1;
												else if(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW]<-400)
													Spin_Flag=-1;
												Filters(Spin_Flag*2000,&ChassisParam.TargetOmega,0.04f);
											}
											else//普通跟随
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
									Speed_Y=Speed_Y>0?Speed_Y-2*SPEED_BUFFER:Speed_Y;
									Speed_Y=Speed_Y<=-Target_VY?-Target_VY:Speed_Y-SPEED_BUFFER;
									if(!CloudParam.Cloud_Gyro.Offline)
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
									Speed_Y=Speed_Y<0?Speed_Y+2*SPEED_BUFFER:Speed_Y-2*SPEED_BUFFER;//缓慢减速
									if(ABS(Speed_Y)<200)
									{
										Speed_Y=0;
									}
								}
								
								if(!DBUS_CheckPush(KEY_F))//没有按下F，正常跟随
								{
									if(!(DBUS_CheckPush(KEY_D)||DBUS_CheckPush(KEY_A)||DBUS_CheckPush(KEY_S)||DBUS_CheckPush(KEY_W)))
									{
										if(!CloudParam.Cloud_Gyro.Offline)
										{
											pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
											if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<1024)
												ChassisParam.TargetOmega=0;	
											else
												Filters(pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW),&ChassisParam.TargetOmega,0.05f);
										}
										else
											ChassisParam.TargetOmega=0;	
									}
								}
								else if(!DBUS_CheckPush(KEY_W))//按下F，且不前进时扭腰
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
								
/************************************************************************************************************************/								
								if(DBUS_CheckPush(KEY_D))
								{
									Speed_X=Speed_X>0?Speed_X-2*SPEED_BUFFER:Speed_X;
									Speed_X=Speed_X<=-Target_VX?-Target_VX:Speed_X-SPEED_BUFFER;
									
									if(!CloudParam.Cloud_Gyro.Offline)
									{
										ChassisParam.TargetOmega=pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
										if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100)
											ChassisParam.TargetOmega=0;
									}
								}
								else if(DBUS_CheckPush(KEY_A))
								{
									Speed_X=Speed_X<0?Speed_X+2*SPEED_BUFFER:Speed_X;
									Speed_X=Speed_X>=Target_VX?Target_VX:Speed_X+SPEED_BUFFER;
									
									if(!CloudParam.Cloud_Gyro.Offline)
									{
										ChassisParam.TargetOmega=pid_calc(&ChassisParam.Chassis_Gyro.Chassis_PID,CloudParam.Yaw.Real_Angle,MEDIAN_YAW);
										if(ABS(ChassisParam.Chassis_Gyro.Chassis_PID.err[NOW])<100)
											ChassisParam.TargetOmega=0;
									}
								}
								else
								{
									Speed_X=Speed_X<0?Speed_X+2*SPEED_BUFFER:Speed_X-2*SPEED_BUFFER;
									if(ABS(Speed_X)<200)
									{
										Speed_X=0;
									}
								}
								
								break ;
/************************************************************************************************************************/
					case    SUPPLY://补给
						
								if(DBUS_CheckPush(KEY_W))
									Speed_Y=SPEED_SUPPLY;	
								
								else if(DBUS_CheckPush(KEY_S))
									Speed_Y=-SPEED_SUPPLY;
								
								else
									Speed_Y=0;
								
								if(DBUS_CheckPush(KEY_A))
									Speed_X=SPEED_SUPPLY;	
								
								else if(DBUS_CheckPush(KEY_D))
									Speed_X=-SPEED_SUPPLY;
								
								else
									Speed_X=0;
								
								if(DBUS_CheckPush(KEY_Q))
									 ChassisParam.TargetOmega=-SPEED_SUPPLY;
								
								else if(DBUS_CheckPush(KEY_E))		
									ChassisParam.TargetOmega=SPEED_SUPPLY;
								
								else
									ChassisParam.TargetOmega=0;
					
								break ;
					case HIEROGRAM://打符
							
								ChassisParam.TargetVY=0;
								ChassisParam.TargetVX=0;
								ChassisParam.TargetOmega=0;	
								break ;
					default:
								break;
				}
				Filters(Speed_X,&ChassisParam.TargetVX,0.1f);//缓冲平滑处理
				Filters(Speed_Y,&ChassisParam.TargetVY,0.1f);

				MecanumCalculate(ChassisParam.TargetVX,ChassisParam.TargetVY,ChassisParam.TargetOmega,WheelSpeed);
				//麦轮解算
				ChassisParam.LF.Target_Speed = WheelSpeed[0];
				ChassisParam.LB.Target_Speed = WheelSpeed[1];
				ChassisParam.RB.Target_Speed = WheelSpeed[2];
				ChassisParam.RF.Target_Speed = WheelSpeed[3];
				M3508_PID_Set();				
			break ;
/************************************************************************************************************************/
		case    Ctrl_OFF://失能控制
			
				ChassisParam.LB.Target_Current=0;
				ChassisParam.LF.Target_Current=0;
				ChassisParam.RB.Target_Current=0;
				ChassisParam.RF.Target_Current=0;
		
			break ;
/************************************************************************************************************************/
	}
	
	Chassis_Current_Set(CAN_1);
}


 /*
  * @brief 麦克纳姆伦运动模型
  * @param Vx X轴方向的速度，Vy Y轴方向的速度，Omega 自旋速度，*Speed轮子转速
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
    
    //限幅
    for(index = 0, MaxSpeed = 0; index < 4; index++)
    {
        if((Buffer[index] > 0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
        {
            MaxSpeed = (Buffer[index] > 0 ? Buffer[index] : -Buffer[index]);//挑出最大的速度
        }
    }	
	
//	if((MEDIAN_ROLL-CloudParam.Cloud_Gyro.Roll)-((CloudParam.Pitch.Real_Angle-(MEDIAN_PITCH-Pitch_Min-200))/22.75f)>12)
//		MaxWheelSpeed=2000;//云台陀螺仪角度变化大于Pitch轴编码器的转换值，被认为是爬坡，把速度降下来
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
  * @brief 底盘功率限制
  * @param None
  * @retval None
  */
void Power_Limit(float Cur_limit)
{
	static float Current;
    //计算四个电机的总电流
	Current=ABS(ChassisParam.LF.Target_Current)+ABS(ChassisParam.LB.Target_Current)\
		   +ABS(ChassisParam.RB.Target_Current)+ABS(ChassisParam.RF.Target_Current);
	
	if(Current>Cur_limit)
	{//根据目标值比例分配电流
		ChassisParam.LF.Target_Current = ChassisParam.LF.Target_Current/Current*Cur_limit;
		ChassisParam.LB.Target_Current = ChassisParam.LB.Target_Current/Current*Cur_limit;
		ChassisParam.RB.Target_Current = ChassisParam.RB.Target_Current/Current*Cur_limit;
		ChassisParam.RF.Target_Current = ChassisParam.RF.Target_Current/Current*Cur_limit;
	}
}

 /*
  * @brief 底盘电机PID计算
  * @param None
  * @retval None
  */
void M3508_PID_Set(void)
{
	//底盘左前轮
	ChassisParam.LF.Target_Current=pid_calc(&ChassisParam.LF.PID,ChassisParam.LF.Real_Speed,ChassisParam.LF.Target_Speed);
	
	//底盘左后轮
	ChassisParam.LB.Target_Current=pid_calc(&ChassisParam.LB.PID,ChassisParam.LB.Real_Speed,ChassisParam.LB.Target_Speed);
	
	//底盘右前轮
	ChassisParam.RF.Target_Current=pid_calc(&ChassisParam.RF.PID,ChassisParam.RF.Real_Speed,ChassisParam.RF.Target_Speed);
	
	//底盘右后轮
	ChassisParam.RB.Target_Current=pid_calc(&ChassisParam.RB.PID,ChassisParam.RB.Real_Speed,ChassisParam.RB.Target_Speed);
}


 /*
* @brief 底盘电机电流发送 ID(0x201~0x204)
  * @param CAN
  * @retval None
  */
void Chassis_Current_Set(CAN_X_State CAN_X)
{
	static  CanSend_Type   SendData;

	SendData.CANx=CAN_X;
	SendData.SendCanTxMsg.StdId = 0x200;
	SendData.SendCanTxMsg.IDE = CAN_ID_STD;
	SendData.SendCanTxMsg.RTR = CAN_RTR_DATA;
	SendData.SendCanTxMsg.DLC = 0x08;
	SendData.SendCanTxMsg.Data[0] = ChassisParam.LB.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[1] = ChassisParam.LB.Target_Current;
	SendData.SendCanTxMsg.Data[2] = ChassisParam.RB.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[3] = ChassisParam.RB.Target_Current;
	SendData.SendCanTxMsg.Data[4] = ChassisParam.RF.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[5] = ChassisParam.RF.Target_Current ;
	SendData.SendCanTxMsg.Data[6] = ChassisParam.LF.Target_Current >> 8;
	SendData.SendCanTxMsg.Data[7] = ChassisParam.LF.Target_Current;
	
	xQueueSend(Queue_CanSend, &SendData, 20);
}
