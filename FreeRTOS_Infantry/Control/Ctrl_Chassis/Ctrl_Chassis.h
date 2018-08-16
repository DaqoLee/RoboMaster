#ifndef __CTRL_CHASSIS_H
#define __CTRL_CHASSIS_H

#include "stm32f4xx_hal.h"
#include "Task_Can.h"

void Set_moto_current(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4);
#endif

