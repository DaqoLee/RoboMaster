#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "Ctrl_Rammer.h"
#include "Ctrl_Chassis.h"
#include "Ctrl_Cloud.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
void CAN_Init(CAN_HandleTypeDef* _hcan);
void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void Analysis_RM_Can(CAN_HandleTypeDef* hcan);

#endif
