#ifndef UART3_H
#define UART3_H

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define UART3_DMA_BUF_SIZE 256

void uart3_init(void);

/*
 * Returns the number of newly received bytes.
 * Copies them into out[].
 */
uint16_t uart3_read(uint8_t *out, uint16_t max_len);
extern TaskHandle_t vn200TaskHandle;

/* Enable the NVIC line. Call from the consuming task's first
 * iteration, never from main - see the note in the .c file. */
void uart3_irq_enable(void);

/* Bytes dropped because the RX ring was full. */
uint32_t uart3_rx_dropped(void);

#endif
