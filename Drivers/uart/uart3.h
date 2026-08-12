#ifndef UART3_H
#define UART3_H

#include <stdint.h>

#define UART3_DMA_BUF_SIZE 256

void uart3_init(void);

/*
 * Returns the number of newly received bytes.
 * Copies them into out[].
 */
uint16_t uart3_read(uint8_t *out, uint16_t max_len);

#endif
