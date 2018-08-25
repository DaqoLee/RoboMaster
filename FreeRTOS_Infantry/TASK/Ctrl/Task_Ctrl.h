#ifndef _TASK_CTRL_H
#define _TASK_CTRL_H
#include "FreeRTOS.h"
#include "task.h"
#include "User_Code.h"

void Task_Control(void *Parameters);
void Control_Mode_Task(void *pvParameters);
#endif
