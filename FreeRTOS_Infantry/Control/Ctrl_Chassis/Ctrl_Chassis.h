#ifndef __CTRL_CHASSIS_H
#define __CTRL_CHASSIS_H

#include "stm32f4xx_hal.h"
#include "Task_Can.h"
#include "User_Code.h"
extern  ChassisParam_Struct   ChassisParam;
void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed);
void M3508_PID_Set(void);
void Chassis_Param_Set(void);
void Power_Limit(float Cur_limit);
void Chassis_Current_Set(CAN_HandleTypeDef* hcan);
#endif

