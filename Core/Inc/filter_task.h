#ifndef FILTER_TASK_H
#define FILTER_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "vn200.h"
#include "dvl.h"
#include "comp_filter.h"


extern QueueHandle_t vn200Queue;
extern QueueHandle_t dvlQueue;
extern QueueHandle_t bar30Queue;


void filter_task(void *argument);

#endif
