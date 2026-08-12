#ifndef DVL_TASK_H
#define DVL_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

void dvl_task(void *argument);

extern QueueHandle_t dvlQueue;

#endif
