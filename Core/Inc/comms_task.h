#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern TaskHandle_t commsTaskHandle;
extern QueueHandle_t commandQueue;

void comms_task(void *argument);

#endif
