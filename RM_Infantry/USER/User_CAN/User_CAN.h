#ifndef	__USER_CAN_H
#define __USER_CAN_H
#include "stm32f4xx_hal.h"
#include "string.h"
typedef union
{
    uint8_t  U[4];
      float  F;
        int  I;
}FormatTrans;


extern uint8_t x,flag;
extern uint8_t Meter_Power_Limit;
void Set_2006_current(CAN_HandleTypeDef* hcan,int16_t Current1);
void Set_6623_current(CAN_HandleTypeDef* hcan,int16_t Current1, int16_t Current2);
void CAN_Init(CAN_HandleTypeDef* _hcan);
void Analysis_RM_Can(CAN_HandleTypeDef* hcan);
void Set_moto_current(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4);
#endif
