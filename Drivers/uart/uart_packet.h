#ifndef UART_PACKET_H
#define UART_PACKET_H

#include <stdint.h>
#include "ring_buffer.h"

#define DMA_BUF_SIZE 256

void uart1_init(void);
void uart1_write_buf(uint8_t *buf, uint16_t len);

/* Enable the NVIC line. Call from the consuming task's first
 * iteration, never from main - see the note in the .c file. */
void uart1_irq_enable(void);

#endif
