#ifndef _TASK_USER_H
#define _TASK_USER_H
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t FLOATTask_Handler;
extern TaskHandle_t LED_TaskHandler;
extern TaskHandle_t KEY_TaskHandler;
extern TaskHandle_t Buzzer_TaskHandler;

void start_task(void *pvParameters);
void led0_task(void *pvParameters);
void float_task(void *pvParameters);
void LED_Task(void *pvParameters);
void KEY_Task(void *pvParameters);
void Buzzer_Task(void *pvParameters);
#endif

