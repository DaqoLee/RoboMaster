#include "User_Code.h"

Game_Mode_State 	Game_Mode=COMMON;
Ctrl_Mode_State 	Control_Mode=Remote_1;
Speed_Mode_State 	Speed_Mode=Normal;
 /*
  * @brief PID������ʼ��
  * @param None
  * @retval None
  */
void PID_Init(void)
{
/********************�������*********************����޷�****�����޷�** P **** I *** D ****/
	PID_struct_init(&M2006.PID.Out,POSITION_PID, M2006_Xianfu,	500,	0.8f,	0,	2.8f);
	PID_struct_init(&M2006.PID.In,DELTA_PID,	 M2006_Xianfu,	500,	0.4f,	0,	   0);
	
/********************���̵��*************************����޷�****�����޷�** P ***** I **** D **/	
    PID_struct_init(&ChassisParam.LF.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	0.5f,	0);
	PID_struct_init(&ChassisParam.LB.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	0.5f,	0);
	PID_struct_init(&ChassisParam.RF.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	0.5f,	0);
	PID_struct_init(&ChassisParam.RB.PID,DELTA_PID,	M3508_Xianfu,	1000,	2.5f,	0.5f,	0);
	
/********************��̨���*********************************����޷�****�����޷�** P **** I *** D */
	PID_struct_init(&CloudParam.Pitch.PID.Out,POSITION_PID,	M6623_Xianfu,	2000,	6.5f,	0,	2.0f);
	PID_struct_init(&CloudParam.Pitch.PID.In,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	0,	1.0f);
	
	PID_struct_init(&CloudParam.Yaw.PID.Out,POSITION_PID,	M6623_Xianfu,	2000,	3.6f,	0,	4.6f);
	PID_struct_init(&CloudParam.Yaw.PID.In,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	0,	1.0f);

/*******************��̨������********************************************����޷�****�����޷�** P ****** I ***** D **/
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.Out,POSITION_PID,	M6623_Xianfu,	500,  100.0f,	   0,  1000.0f);
	PID_struct_init(&CloudParam.Cloud_Gyro.Pitch_PID.In,POSITION_PID,	M6623_Xianfu,	1000,	1.2f,   0.1f,	  2.0f);
	
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.Out,POSITION_PID,	M6623_Xianfu,	500,  120.0f,	   0,	100.0f);
	PID_struct_init(&CloudParam.Cloud_Gyro.Yaw_PID.In ,POSITION_PID,	M6623_Xianfu,	2000,	0.8f,	   0,	  1.0f);

}

 /*
  * @brief ��ͬģʽ��̨PID����
  * @param modeΪ��ǰģʽ
  * @retval None
  */
void PID_REST(Game_Mode_State mode)
{
	static uint8_t Mode_flag=0;
	switch(mode)
	{
		case    COMMON://����
				if(Mode_flag!=mode)//                                                 P     I   D
				{
					CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,		2.2f,	0,  0);
					CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,	6.5f,	0,	0);
					Mode_flag=mode;
				}
				break ;
		case    SUPPLY://����
				if(Mode_flag!=mode)
				{
					CloudParam.Yaw.PID.Out.f_pid_reset(&CloudParam.Yaw.PID.Out,		2.0f,	0,	0);
					CloudParam.Pitch.PID.Out.f_pid_reset(&CloudParam.Pitch.PID.Out,	6.5f,	0,	0);	
					Mode_flag=mode;
				}
				break ;
		case HIEROGRAM://���
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
  * @brief ���������������ģʽ����
  * @param None
  * @retval None
  */
void Game_Mode_Set(void)
{
    if(DBUS_CheckPush(KEY_CTRL)&&DBUS_CheckPush(KEY_C))
		Game_Mode=COMMON;
	
	else if(DBUS_CheckPush(KEY_SHIFT)&&DBUS_CheckPush(KEY_X))
		Game_Mode=HIEROGRAM;
	
	else if(DBUS_CheckPush(KEY_CTRL)&&DBUS_CheckPush(KEY_X))
		Game_Mode=SUPPLY;
	
}

 /*
  * @brief ң�أ�����ģʽ����
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
				Control_Mode=Ctrl_OFF;//��ʧ�ܿ���ʱ����ͨ�����ѡ��ң��ʱ�����Ƿ������̨
				switch(DBUS_ReceiveData.switch_left)
				{
					case    1:
						
						break ;
					case    2:
						if(Flag==1)//������л�ģʽ(�С���)
						{
							Flag=0;
							Mode_flag++;
							Mode_flag=Mode_flag>2?1:Mode_flag;//ģʽ��1��2֮��
							
						}
						break ;
					case    3:
							Flag=1;
							for(i=1;i<=Mode_flag;i++)//ÿ��800ms����������1��Ϊģʽ1��2��Ϊģʽ2��
							{
								TIM12->ARR=1899;
								vTaskDelay(100);
								TIM12->ARR=0;
								vTaskDelay(100);
							}
							vTaskDelay(800);
							i=1;
						break ;
				}
				break ;
				
		case 	3:
				switch(Mode_flag)//�Ҽ����м��л���Ĭ����1
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
				break;
	}
}

 /*
  * @brief ��ϰ���
  * @param None
  * @retval None
  */
void Key_Combination(void)
{
	static uint8_t Flag=0;
	switch(DBUS_ReceiveData.keyBoard.key_code)
	{
		case	KEY_SHIFT|KEY_X://���
			
					if(Flag==1)
					{
						Game_Mode=HIEROGRAM;
						Flag=0;
					}
					break ;
		case	KEY_CTRL|KEY_X://����
			
					if(Flag==1)
					{
						Game_Mode=SUPPLY;
						Flag=0;
					}		
					break ;
		case	KEY_CTRL|KEY_C://����
			
					if(Flag==1)
					{
						Game_Mode=COMMON;
						Flag=0;
					}
					break ;
		case	KEY_CTRL|KEY_W:
			
						Speed_Mode=UP;
					break ;
		case	KEY_SHIFT|KEY_V:
			
						Speed_Mode=Down;
					break ;
		case	KEY_CTRL|KEY_V://��/��Ħ����
			
					if(Flag==1)
					{
						Frict_OFF=!Frict_OFF;
						Flag=0;
					}
					break ;
		case	KEY_SHIFT|KEY_R://��������ʹ��/ʧ��
				
					if(Flag==1)
					{
						CAP_OFF=!CAP_OFF;
						Flag=0;
					}
					break ;
		case	KEY_SHIFT|KEY_C:
			
					GPIOG->BSRR=1<<3;
					break ;
		default:
					Flag=1;
					Speed_Mode=Normal;	
					break;
	}
}


void Moto_Current_Set(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
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
 /*
  * @brief �˲�����������ƽ������
  * @param Val �˲�ǰ��*Value�˲���
  * @retval None
  */
void Filters(int16_t Val,float *Value,float Rate)
{
	*Value+=Rate*(Val-*Value);
}

