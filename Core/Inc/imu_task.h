#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sensor_types.h"

/*
 * imu_task: reads the VN-200 (USART3) and the BNO085 (I2C1), and
 * publishes one ImuSample from whichever is the active source. VN-200
 * preferred, BNO085 backup; see source_select.h for the rule.
 *
 * imuQueue holds one ImuSample, written with xQueueOverwrite and read with
 * xQueuePeek, so readers always get the newest and never wait.
 */
extern QueueHandle_t imuQueue;

/* Notified by the USART3 interrupt (uart3.c) when VN-200 bytes arrive. */
extern TaskHandle_t imuTaskHandle;

void imu_task(void *argument);

#endif
