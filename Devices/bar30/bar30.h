#ifndef BAR30_TASK_H
#define BAR30_TASK_H

#include "FreeRTOS.h"
#include "queue.h"


extern QueueHandle_t bar30Queue;


void bar30_task(void *argument);

#endif
