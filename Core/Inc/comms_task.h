#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern TaskHandle_t commsTaskHandle;
extern QueueHandle_t commandQueue;

void comms_task(void *argument);

/* Parsed commands dropped because commandQueue was full. Non-zero means
 * control_task is not draining. */
uint32_t comms_cmd_drops(void);

/* CRC-valid command packets received since boot, monotonic. */
uint32_t comms_cmd_valid(void);

#endif
