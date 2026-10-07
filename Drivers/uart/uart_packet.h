#ifndef UART_PACKET_H
#define UART_PACKET_H

#include <stdint.h>
#include "ring_buffer.h"

#define DMA_BUF_SIZE 256

#include <stdbool.h>

void uart1_init(void);

/* Non-blocking: hand a frame to the TX DMA. False if one is in flight.
 * Completion is signalled to comms_task as COMMS_NOTIFY_TX_DONE. */
bool uart1_tx_dma_start(const uint8_t *buf, uint16_t len);

/* Blocking, bench use only. Waits for any DMA frame to finish first. */
void uart1_write_buf(uint8_t *buf, uint16_t len);

/* Enable the NVIC line. Call from the consuming task's first
 * iteration, never from main - see the note in the .c file. */
void uart1_irq_enable(void);

#endif
