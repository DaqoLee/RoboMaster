#ifndef _TASK_CAN_H
#define _TASK_CAN_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "BSP_CAN.h"

extern xQueueHandle    Queue_CanSend; 
void Task_CanSend(void *Parameters);

#endif

