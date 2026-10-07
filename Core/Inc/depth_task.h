#ifndef DEPTH_TASK_H
#define DEPTH_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "sensor_types.h"

/*
 * depth_task: reads the Bar30 (I2C1) and the analog pressure sensor
 * (ADC1, PA4) and publishes one DepthSample from whichever is the active
 * source. Bar30 preferred, analog backup; see source_select.h.
 *
 * depthQueue holds one DepthSample, written with xQueueOverwrite and read
 * with xQueuePeek.
 */
extern QueueHandle_t depthQueue;

void depth_task(void *argument);

#endif
