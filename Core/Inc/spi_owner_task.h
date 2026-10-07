#ifndef SPI_OWNER_TASK_H
#define SPI_OWNER_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include <stdint.h>

/*
 * The only task that uses SPI2 (SD card and TFT) once the scheduler runs.
 * Others send it requests. The two devices need different SPI clocks, and
 * with one owner the clock is set in one place before every transfer.
 * (A mutex would be better if a task at priority 4+ ever needed the bus
 * directly; none does.)
 */

typedef enum
{
    SPI_REQ_SD_BLOCK = 0,     /* write block[] at block_addr */
    SPI_REQ_DISPLAY  = 1      /* redraw the status screen now */
} SpiRequestType;

#define SPI_BLOCK_BYTES   512U

/* Passed by value, block included, so no buffer is shared between tasks. */
typedef struct
{
    SpiRequestType type;

    uint32_t block_addr;
    uint8_t  block[SPI_BLOCK_BYTES];
} SpiRequest;

extern QueueHandle_t spiRequestQueue;

void spi_owner_task(void *argument);

uint32_t spi_owner_sd_writes(void);
uint32_t spi_owner_sd_errors(void);

#endif /* SPI_OWNER_TASK_H */
