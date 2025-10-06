#ifndef __INIT_TASK_H__
#define __INIT_TASK_H__

#include "HeadFiles.h"
extern SemaphoreHandle_t rtc_mutex;
void Init_Task(void* pvParament);

#endif // __INIT_TASK_H__