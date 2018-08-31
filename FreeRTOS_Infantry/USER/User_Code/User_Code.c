#include "User_Code.h"

Game_Mode_State 	Game_Mode=COMMON;
Ctrl_Mode_State 	Control_Mode=Remote_1;
Speed_Mode_State 	Speed_Mode=Normal;
 /*
  * @brief PID参数初始化
  * @param None
  * @retval None
  */
void PID_Init(void)
{
/********************拨弹电机*********************输出限幅****积分限幅** P **** I *** D ****/
	PID_struct_init(&M2006.PID.Out,POSITION_PID, M2006_Xianfu,	500,	0.8f,	0,	2.8f);
	PID_struct_init(&M2006.PID.In,DELTA_PID,	 M2006_Xianfu,	500,	0.4f,	0,	   0);
	
/********************底盘电机*************************输出限幅****积分限幅** P ***** I **** D **/	
    PID_struct_init(&ChassisParam.LF.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	1.0f,	0);
	PID_struct_init(&ChassisParam.LB.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	1.0f,	0);
	PID_struct_init(&ChassisParam.RF.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	1.0f,	0);
	PID_struct_init(&ChassisParam.RB.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	1.0f,	0);
	
	PID_struct_init(&ChassisParam.Chassis_Gyro.Chassis_PID,POSITION_PID,4000,1000,3.5f,0,0);
/********************云台电机*********************************输出限幅****积分限幅** P **** I *** D */
	PID_struct_init(&CloudParam.Pitch.PID.Out,POSITION_PID,	M6623_Xianfu,	2000,	10.5f,	0,	0.0f);
	PID_struct_init(&CloudParam.Pitch.PID.In,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	0,	0.5f);
	
	PID_struct_init(&CloudParam.Yaw.PID.Out,POSITION_PID,	M6623_Xianfu,	2000,	4.6f,	0,	0.0f);
	PID_struct_init(&CloudParam.Yaw.PID.In,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	0,	1.0f);
	
/*******************云台陀螺仪********************************************输出限幅****积分限幅** P ****** I ***** D **/
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.Out,POSITION_PID,	M6623_Xianfu,	500,  100.0f,	   0,  1000.0f);
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.In,POSITION_PID,	M6623_Xianfu,	1000,	1.2f,   0.1f,	  2.0f);
	
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.Out,POSITION_PID,	M6623_Xianfu,	500,    200.0f,	   0,	100.0f);
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.In ,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	   0,	  1.0f);

}

 /*
  * @brief 不同模式云台PID设置
  * @param mode为当前模式
  * @retval None
  */
void PID_REST(Game_Mode_State mode)
{
	static uint8_t Mode_flag=0;
	switch(mode)
	{
		case    COMMON://一般
				if(Mode_flag!=mode)//                                                 P     I   D
				{
					CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,		2.2f,	0,  0);
					CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,	10.5f,	0,	0);
					Mode_flag=mode;
				}
				break ;
		case    SUPPLY://补给
				if(Mode_flag!=mode)
				{
					CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,		2.0f,	0,	0);
					CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,	2.5f,	0,	0);	
					Mode_flag=mode;
				}
				break ;
		case HIEROGRAM://打符
				if(Mode_flag!=mode)
				{
					CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,		1.2f,	0,	0);
					CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,	5.5f,	0,	0);
					Mode_flag=mode;
				}
				break ;
		default:
				break;
	}
}
 /*
  * @brief 获取初始角度
  * @param None
  * @retval None
  */
void Get_Target_Angle(void)
{
	ChassisParam.TargetABSAngle=ChassisParam.Chassis_Gyro.Yaw;
	ChassisParam.Chassis_Gyro.Target_Yaw=ChassisParam.Chassis_Gyro.Yaw;
	CloudParam.Cloud_Gyro.Target_Yaw=CloudParam.Cloud_Gyro.Yaw;
	CloudParam.Cloud_Gyro.Target_Roll=CloudParam.Cloud_Gyro.Roll;
	ERror=ChassisParam.Chassis_Gyro.Yaw-CloudParam.Cloud_Gyro.Yaw;
	CloudParam.Yaw.Target_Angle=CloudParam.Yaw.Real_Angle;
	CloudParam.Pitch.Target_Angle=CloudParam.Pitch.Real_Angle;
}

 /*
  * @brief 遥控，键鼠模式设置
  * @param None
  * @retval None
  */
void Control_Mode_Set(void)
{
	static uint8_t Mode_flag=1,Flag=0,i;
	switch(DBUS_ReceiveData.switch_right)
	{
		case    1:
				Control_Mode=Keyboard;
				break ;
		case    2:
				Control_Mode=Ctrl_OFF;//在失能控制时，可以通过左键选择遥控时底盘是否跟随云台
				switch(DBUS_ReceiveData.switch_left)
				{
					case    1:
						
						break ;
					case    2:
						if(Flag==1)//拨左键切换模式(中→下)
						{
							Flag=0;
							Mode_flag++;
							Mode_flag=Mode_flag>2?1:Mode_flag;//模式在1和2之间
							
						}
						break ;
					case    3:
							Flag=1;
							for(i=1;i<=Mode_flag;i++)//每隔800ms蜂鸣器鸣叫1声为模式1，2声为模式2；
							{
								TIM12->ARR=1899;
								HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,GPIO_PIN_RESET);
								vTaskDelay(100);
								TIM12->ARR=0;
								HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_SET);
								HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,GPIO_PIN_SET);
								vTaskDelay(100);
								
							}
							vTaskDelay(800);
							i=1;
						break ;
				}
				break ;
				
		case 	3:
				switch(Mode_flag)//右键在中间切换，默认是1
				{
					case    1:
						Control_Mode=Remote_1;
						break ;
					case    2:
						Control_Mode=Remote_2;
						break ;
				}
				break ;
		default:
				Control_Mode=Ctrl_OFF;
				break;
	}
}

 /*
  * @brief 组合按键
  * @param None
  * @retval None
  */
void Key_Combination(void)
{
	static uint8_t Flag=0;
	switch(DBUS_ReceiveData.keyBoard.key_code)
	{
		case	KEY_SHIFT|KEY_X://打符
			
					if(Flag==1)
					{
						Game_Mode=HIEROGRAM;
						Flag=0;
					}
					break ;
		case	KEY_CTRL|KEY_X://补给
			
					if(Flag==1)
					{
						Game_Mode=SUPPLY;
						Flag=0;
					}		
					break ;
		case	KEY_CTRL|KEY_C://正常
			
					if(Flag==1)
					{
						Game_Mode=COMMON;
						Flag=0;
					}
					break ;
		case	KEY_SHIFT|KEY_W:
		case	KEY_SHIFT|KEY_S:
		case	KEY_SHIFT|KEY_A:
		case	KEY_SHIFT|KEY_D:
			
						Speed_Mode=UP;
					break ;
		case	KEY_CTRL|KEY_W:
		case	KEY_CTRL|KEY_S:
		case	KEY_CTRL|KEY_A:
		case	KEY_CTRL|KEY_D:
			
						Speed_Mode=Down;
					break ;
		case	KEY_CTRL|KEY_V://开/关摩擦轮
			
					if(Flag==1)
					{
						Frict_OFF=!Frict_OFF;
						Flag=0;
					}
					break ;
		case	KEY_SHIFT|KEY_R://超级电容使能/失能
				
					if(Flag==1)
					{
						CAP_OFF=!CAP_OFF;
						Flag=0;
					}
					break ;
		case	KEY_SHIFT|KEY_C:
			
					break ;
		default:
					Flag=1;
					Speed_Mode=Normal;	
					break;
	}
}


void Moto_Current_Set(CAN_X_State CAN_X,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
{
	static  CanSend_Type   SendData;

	SendData.CANx=CAN_X;
	SendData.SendCanTxMsg.StdId = ID;
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
 /*
  * @brief 滤波函数，数据平滑处理
  * @param Val 滤波前，*Value滤波后
  * @retval None
  */
void Filters(int16_t Val,float *Value,float Rate)
{
	*Value+=Rate*(Val-*Value);
}
 /*
  * @brief 外设报文返回帧率统计，频率10Hz
  * @param None
  * @retval None
  */
void Frame_Rate_Statistics(void)//帧率统计100ms一次
{

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
		//HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);
	}
	else
	{
		CloudParam.Cloud_Gyro.Offline=0;
		//HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);
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
	
//	if(Judge.FrameRate==0)
//	{
//		Judge.Offline=1;
//		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);
//	}
//	else
//	{
//		Judge.Offline=0;
//		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET);
//	}
//	
//	if(Remote.FrameRate==0)
//		Remote.Offline=1;
//	else
//		Remote.Offline=0;
//	
//	if(PC_Data.FrameRate==0)
//		PC_Data.Offline=1;
//	else
//		PC_Data.Offline=0;
//	
//	if(Cur_Meter.FrameRate==0)
//	{
//		Cur_Meter.Offline=1;
//		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_RESET);
//	}
//	else
//	{
//		Cur_Meter.Offline=0;
//		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_SET);
//	}
/******************************************************************/	
	
	CloudParam.Pitch.FrameRate=0;
	CloudParam.Yaw.FrameRate=0;
	CloudParam.Cloud_Gyro.FrameRate=0;
	
	ChassisParam.LB.FrameRate=0;
	ChassisParam.LF.FrameRate=0;
	ChassisParam.RB.FrameRate=0;
	ChassisParam.RF.FrameRate=0;
	ChassisParam.Chassis_Gyro.FrameRate=0;
	
	M2006.FrameRate=0;
//	Judge.FrameRate=0;
//	Remote.FrameRate=0;
//	PC_Data.FrameRate=0;
//	Cur_Meter.FrameRate=0;

}

