#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "sd_logger.h"

/*
 * Phase 8 — display_task is the SOLE owner of SPI1.
 *
 * Neither oled.c nor sd_card.c have any built-in locking, and
 * sd_write_block() ends in an unbounded busy-wait for the card to
 * clear its busy state (see sd_card.c). Rather than guard every
 * call site with a mutex (easy to forget in raw driver files),
 * only this task is ever allowed to call the spi_, oled_, and sd_
 * functions. Every other task that needs the bus goes through
 * spiRequestQueue instead - locking is structural, not disciplined.
 */

typedef enum
{
    SPI_REQ_SD_LOG

} SpiRequestType;

typedef struct
{
    SpiRequestType type;
    LogRecord      record;

} SpiRequest;

extern QueueHandle_t spiRequestQueue;

void display_task(void *argument);

#endif
