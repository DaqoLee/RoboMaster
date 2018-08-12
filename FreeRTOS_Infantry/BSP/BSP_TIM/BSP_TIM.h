#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim12;

void TIM_Init(void);
void MX_TIM1_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_TIM12_Init(void);
                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif



