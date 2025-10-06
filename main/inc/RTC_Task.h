#ifndef __RTC_TASK_H__
#define __RTC_TASK_H__
#include "HeadFiles.h"

extern TaskHandle_t RTC_Update_Task_Handle;



void RTC_Update_Task(void *pvParameters);
void RTC_Task(void *pvParameters);
#endif // __RTC_TASK_H__