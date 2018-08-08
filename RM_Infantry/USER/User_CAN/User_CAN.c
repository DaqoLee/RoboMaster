#include "User_can.h"
#include "can.h"
#include "control.h"
#include "Ctrl_Rammer.h"
FormatTrans   Type;//类型转换
FrameRate_Struct Cur_Meter;
uint8_t Meter_Power_Limit;
 /*
  * @brief CAN初始化
  * @param * hcan CAN结构体指针
  * @retval None
  */
void CAN_Init(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		Tx1Message;
	static CanRxMsgTypeDef 		Rx1Message;
	static CanTxMsgTypeDef		Tx2Message;
	static CanRxMsgTypeDef 		Rx2Message;

	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;//
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}

	//filter config for can2 
	//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterNumber = 14;
	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
     	//err_deadloop();
	}

	if(_hcan == &hcan1)
	{
		_hcan->pTxMsg = &Tx1Message;
		_hcan->pRxMsg = &Rx1Message;
	}

	if(_hcan == &hcan2)
	{
		_hcan->pTxMsg = &Tx2Message;
		_hcan->pRxMsg = &Rx2Message;
	}
	 __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);
	 __HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);
}

 /*
  * @brief CAN报文解析
  * @param * hcan CAN结构体指针
  * @retval None
  */
void Analysis_RM_Can(CAN_HandleTypeDef* hcan)//报文解析
{
	uint8_t i;
	switch(hcan->pRxMsg->StdId)
	{
		case 0x201:
			ChassisParam.LB.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
		    ChassisParam.LB.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
			ChassisParam.LB.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
		    ChassisParam.LB.FrameRate++;
			break;
		case 0x202:
			ChassisParam.RB.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
		    ChassisParam.RB.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
			ChassisParam.RB.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
			ChassisParam.RB.FrameRate++;
			break;
		case 0x203:
			ChassisParam.RF.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
		    ChassisParam.RF.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
			ChassisParam.RF.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
		    ChassisParam.RF.FrameRate++;
			break;
		case 0x204:
			ChassisParam.LF.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
		    ChassisParam.LF.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
			ChassisParam.LF.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
			ChassisParam.LF.FrameRate++;
			break;
		case 0x205:
			CloudParam.Yaw.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
			CloudParam.Yaw.Real_Current=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
			CloudParam.Yaw.FrameRate++;
			break;
		case 0x206:
			CloudParam.Pitch.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
			CloudParam.Pitch.Real_Current=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
			CloudParam.Pitch.FrameRate++;
			break;
		case 0x207:
			M2006.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
			M2006.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
		    M2006.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
		    Rammer_Angle();
			M2006.FrameRate++;
		
		    break;
		case 0x208:
			break;
		
//		case 0x259:	//UWB定位标签
//			
//       if(hcan->pRxMsg->DLC==8)
//			{
//			  flag++;
//				if(flag==1)
//				{
//					UWB.X=((int16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0]);
//					UWB.Y=((int16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2]);
//					UWB.Angle=((uint16_t)(hcan->pRxMsg->Data[5]<<8)|hcan->pRxMsg->Data[4])/100;
//					UWB.Distance[0]=((int16_t)(hcan->pRxMsg->Data[7]<<8)|hcan->pRxMsg->Data[6]);
//					
//				}	
//				else if(flag==2)
//				{
//					
//					UWB.Distance[1]=((int16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0]);
//					UWB.Distance[2]=((int16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2]);
//					UWB.Distance[3]=((int16_t)(hcan->pRxMsg->Data[5]<<8)|hcan->pRxMsg->Data[4]);
//					UWB.Distance[4]=((int16_t)(hcan->pRxMsg->Data[7]<<8)|hcan->pRxMsg->Data[6]);
//				}
//				
//			}
//			
//			if(hcan->pRxMsg->DLC==6)
//			{    flag=0;
//				 UWB.Distance[5]=((int16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0]);
//				 UWB.Error=((int16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2]);
//					
//			}
//		
//			break;
		case 0x11://云台陀螺仪
			if(hcan->pRxMsg->DLC==4)
			{
				for(i=0;i<4;i++)
				{
				   Type.U[i]=(int16_t)(hcan->pRxMsg->Data[i]);
				}
				   CloudParam.Gyro.radian=Type.F;
				   CloudParam.Gyro.angle=57.3f*CloudParam.Gyro.radian;
			}
		    CloudParam.Gyro.FrameRate++;
			break;
		case 0x401://电流计
			
		//	memcpy(Current_Meter.data.dataBuff,hcan->pRxMsg->Data,sizeof(uint8_t[8]));
		    Current_Meter.data.ChassisVolt=((uint16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0])/100.0f;
		    Current_Meter.data.ChassisCurrent=((uint16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2])/100.0f;
		    Current_Meter.data.ChassisPower=((uint16_t)(hcan->pRxMsg->Data[5]<<8)|hcan->pRxMsg->Data[4])/100.0f;
            Current_Meter.data.ChassisPowerBuffer=((uint16_t)(hcan->pRxMsg->Data[7]<<8)|hcan->pRxMsg->Data[6])/100.0f;
		
			if(Current_Meter.data.ChassisPowerBuffer<30)
				Meter_Power_Limit=1;
			else if(Current_Meter.data.ChassisPowerBuffer<15)
				Meter_Power_Limit=2;
			else if(Current_Meter.data.ChassisPowerBuffer>50)
				Meter_Power_Limit=0;
		    Cur_Meter.FrameRate++;
			break;
	
				
		default:
			break;
	
	}

}

 /*
  * @brief CAN中断回调函数
  * @param * hcan CAN结构体指针
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	
	if(_hcan==&hcan1)
	{
	    Analysis_RM_Can(&hcan1);//CAN1解析
	    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	}
	if(_hcan==&hcan2)
	{
	    Analysis_RM_Can(&hcan2);//CAN2解析
	    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
	}
	
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
	
	HAL_CAN_Transmit(hcan, 1000);
}	

void Set_6623_current(CAN_HandleTypeDef* hcan,int16_t Current1, int16_t Current2)
{

	hcan->pTxMsg->StdId = 0x1FF;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = Current1 >> 8;
	hcan->pTxMsg->Data[1] = Current1;
	hcan->pTxMsg->Data[2] = Current2 >> 8;
	hcan->pTxMsg->Data[3] = Current2;

	HAL_CAN_Transmit(hcan, 1000);
}	
void Set_2006_current(CAN_HandleTypeDef* hcan,int16_t Current1)
{

	hcan->pTxMsg->StdId = 0x1FF;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[4] = Current1 >> 8;
	hcan->pTxMsg->Data[5] = Current1;

	HAL_CAN_Transmit(hcan, 1000);
}	


