#ifndef VN200_TASK_H
#define VN200_TASK_H

#include "FreeRTOS.h"
#include "queue.h"


extern QueueHandle_t vn200Queue;


void vn200_task(void *argument);

#endif
