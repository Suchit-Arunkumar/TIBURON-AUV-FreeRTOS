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

#endif
