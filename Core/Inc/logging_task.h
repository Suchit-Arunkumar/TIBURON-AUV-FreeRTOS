#ifndef LOGGING_TASK_H
#define LOGGING_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "sd_logger.h"

#include <stdint.h>

/*
 * Records are packed into 512-byte blocks and written when a block is full
 * (every ~2.4 s) or on a flush. One block per record would program 12.8x
 * more flash and hit the card's 250 ms worst-case busy time 5 times a
 * second. Block layout: sd_logger.h.
 */
/* What goes on logQueue. A flush is a queue item, not a notification, so
 * it stays in order with the records before it. */
typedef enum
{
    LOG_ITEM_RECORD = 0,
    LOG_ITEM_FLUSH  = 1
} LogItemKind;

typedef struct
{
    uint8_t   kind;
    LogRecord record;   /* meaningful only when kind == LOG_ITEM_RECORD */
} LogQueueItem;

extern QueueHandle_t logQueue;

void logging_task(void *argument);

/* Counters for the Phase 9 console report. */
uint32_t logging_blocks_emitted(void);
uint32_t logging_records_staged(void);
uint32_t logging_spi_post_drops(void);

#endif
