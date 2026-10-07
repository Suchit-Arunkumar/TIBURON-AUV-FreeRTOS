#ifndef UART4_H
#define UART4_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define UART4_DMA_BUF_SIZE 256U

extern TaskHandle_t dvlTaskHandle;

void uart4_init(void);

uint16_t uart4_read(
    uint8_t *out,
    uint16_t max_len
);

/* Enable the NVIC line. Call from the consuming task's first
 * iteration, never from main - see the note in the .c file. */
void uart4_irq_enable(void);

/* Bytes dropped because the RX ring was full. */
uint32_t uart4_rx_dropped(void);

#endif
