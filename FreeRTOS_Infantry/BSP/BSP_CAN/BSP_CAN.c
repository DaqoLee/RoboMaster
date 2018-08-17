#include "BSP_CAN.h"

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

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
		MX_CAN1_Init();
		_hcan->pTxMsg = &Tx1Message;
		_hcan->pRxMsg = &Rx1Message;
		
	}

	if(_hcan == &hcan2)
	{
		MX_CAN2_Init();
		_hcan->pTxMsg = &Tx2Message;
		_hcan->pRxMsg = &Rx2Message;
	}
	 __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);
	 __HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);
}

/*
  * @brief CAN
  * @param * hcan CAN
  * @retval None
  */
void Analysis_RM_Can(CAN_HandleTypeDef* hcan)
{
//	uint8_t i;
	switch(hcan->pRxMsg->StdId)
	{
//		case 0x201:
//			ChassisParam.LB.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
//		    ChassisParam.LB.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
//			ChassisParam.LB.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
//		    ChassisParam.LB.FrameRate++;
//			break;
//		case 0x202:
//			ChassisParam.RB.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
//		    ChassisParam.RB.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
//			ChassisParam.RB.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
//			ChassisParam.RB.FrameRate++;
//			break;
//		case 0x203:
//			ChassisParam.RF.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
//		    ChassisParam.RF.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
//			ChassisParam.RF.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
//		    ChassisParam.RF.FrameRate++;
//			break;
//		case 0x204:
//			ChassisParam.LF.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
//		    ChassisParam.LF.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
//			ChassisParam.LF.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
//			ChassisParam.LF.FrameRate++;
//			break;
//		case 0x205:
//			CloudParam.Yaw.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
//			CloudParam.Yaw.Real_Current=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
//			CloudParam.Yaw.FrameRate++;
//			break;
//		case 0x206:
//			CloudParam.Pitch.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
//			CloudParam.Pitch.Real_Current=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
//			CloudParam.Pitch.FrameRate++;
//			break;
		case 0x207:
			M2006.Real_Angle=((int16_t)(hcan->pRxMsg->Data[0]<<8)|hcan->pRxMsg->Data[1]);
			M2006.Real_Speed=((int16_t)(hcan->pRxMsg->Data[2]<<8)|hcan->pRxMsg->Data[3]);
		    M2006.Real_Current=((int16_t)(hcan->pRxMsg->Data[4]<<8)|hcan->pRxMsg->Data[5]);
		    Rammer_Angle();
			M2006.FrameRate++;
		
		    break;
//		case 0x208:
//			break;
//		
////		case 0x259:	//UWB????
////			
////       if(hcan->pRxMsg->DLC==8)
////			{
////			  flag++;
////				if(flag==1)
////				{
////					UWB.X=((int16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0]);
////					UWB.Y=((int16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2]);
////					UWB.Angle=((uint16_t)(hcan->pRxMsg->Data[5]<<8)|hcan->pRxMsg->Data[4])/100;
////					UWB.Distance[0]=((int16_t)(hcan->pRxMsg->Data[7]<<8)|hcan->pRxMsg->Data[6]);
////					
////				}	
////				else if(flag==2)
////				{
////					
////					UWB.Distance[1]=((int16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0]);
////					UWB.Distance[2]=((int16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2]);
////					UWB.Distance[3]=((int16_t)(hcan->pRxMsg->Data[5]<<8)|hcan->pRxMsg->Data[4]);
////					UWB.Distance[4]=((int16_t)(hcan->pRxMsg->Data[7]<<8)|hcan->pRxMsg->Data[6]);
////				}
////				
////			}
////			
////			if(hcan->pRxMsg->DLC==6)
////			{    flag=0;
////				 UWB.Distance[5]=((int16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0]);
////				 UWB.Error=((int16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2]);
////					
////			}
////		
////			break;
//		case 0x11://?????
//			if(hcan->pRxMsg->DLC==4)
//			{
//				for(i=0;i<4;i++)
//				{
//				   Type.U[i]=(int16_t)(hcan->pRxMsg->Data[i]);
//				}
//				   CloudParam.Gyro.radian=Type.F;
//				   CloudParam.Gyro.angle=57.3f*CloudParam.Gyro.radian;
//			}
//		    CloudParam.Gyro.FrameRate++;
//			break;
//		case 0x401://???
//			
//		//	memcpy(Current_Meter.data.dataBuff,hcan->pRxMsg->Data,sizeof(uint8_t[8]));
//		    Current_Meter.data.ChassisVolt=((uint16_t)(hcan->pRxMsg->Data[1]<<8)|hcan->pRxMsg->Data[0])/100.0f;
//		    Current_Meter.data.ChassisCurrent=((uint16_t)(hcan->pRxMsg->Data[3]<<8)|hcan->pRxMsg->Data[2])/100.0f;
//		    Current_Meter.data.ChassisPower=((uint16_t)(hcan->pRxMsg->Data[5]<<8)|hcan->pRxMsg->Data[4])/100.0f;
//            Current_Meter.data.ChassisPowerBuffer=((uint16_t)(hcan->pRxMsg->Data[7]<<8)|hcan->pRxMsg->Data[6])/100.0f;
//		
//			if(Current_Meter.data.ChassisPowerBuffer<30)
//				Meter_Power_Limit=1;
//			else if(Current_Meter.data.ChassisPowerBuffer<15)
//				Meter_Power_Limit=2;
//			else if(Current_Meter.data.ChassisPowerBuffer>50)
//				Meter_Power_Limit=0;
//		    Cur_Meter.FrameRate++;
//			break;
	
				
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
			Analysis_RM_Can(&hcan1);
	    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	}
	if(_hcan==&hcan2)
	{
	   
	    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
	}
	
}
/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_4TQ;
  hcan1.Init.BS1 = CAN_BS1_2TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_4TQ;
  hcan2.Init.BS1 = CAN_BS1_2TQ;
  hcan2.Init.BS2 = CAN_BS2_2TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan2);

}

static int HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral interrupt init */
	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
 
  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
} 
