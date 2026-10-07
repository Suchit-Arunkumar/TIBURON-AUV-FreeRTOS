#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern TaskHandle_t commsTaskHandle;
extern QueueHandle_t commandQueue;

/*
 * comms_task wakes on task-notification bits, one per event:
 *   RX         USART1 RX DMA moved bytes into the ring (uart_packet.c ISR)
 *   TELEMETRY  control_task finished a tick, send a telemetry frame
 *   TX_DONE    the DMA finished sending a frame (uart_packet.c ISR)
 * Bits are latched until comms_task reads them, so two events arriving
 * before it runs are both seen.
 */
#define COMMS_NOTIFY_RX          (1UL << 0)
#define COMMS_NOTIFY_TELEMETRY   (1UL << 1)
#define COMMS_NOTIFY_TX_DONE     (1UL << 2)

void comms_task(void *argument);

/* Parsed commands dropped because commandQueue was full. Non-zero means
 * control_task is not draining. */
uint32_t comms_cmd_drops(void);

/* CRC-valid command packets received since boot, monotonic. */
uint32_t comms_cmd_valid(void);

/* Outgoing frames dropped because the TX ring was full. */
uint32_t comms_tx_drops(void);

#endif
