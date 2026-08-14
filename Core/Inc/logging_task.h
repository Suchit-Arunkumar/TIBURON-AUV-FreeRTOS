#ifndef LOGGING_TASK_H
#define LOGGING_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t logQueue;

void logging_task(void *argument);

#endif
