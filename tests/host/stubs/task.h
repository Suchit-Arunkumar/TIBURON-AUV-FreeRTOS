#ifndef TASK_H
#define TASK_H

#include "FreeRTOS.h"

static inline TickType_t xTaskGetTickCount(void) { return host_tick; }

#endif
