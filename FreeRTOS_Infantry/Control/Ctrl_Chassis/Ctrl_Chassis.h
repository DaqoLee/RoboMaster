#ifndef __CTRL_CHASSIS_H
#define __CTRL_CHASSIS_H

#include "stm32f4xx_hal.h"
#include "Task_Can.h"

extern  ChassisParam_Struct   ChassisParam;
void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed);
void M3508_PID_Set(void);
void Chassis_Param_Set(uint16_t Max_X,uint16_t Max_Y);
void Power_Limit(float Cur_limit);
void Set_moto_current(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4);
#endif

