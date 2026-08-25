#ifndef LOGGING_TASK_H
#define LOGGING_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "sd_logger.h"

#include <stdint.h>

/*
 * Record batching.
 *
 * A LogRecord is 40 bytes and control_task emits one every 10th tick, so
 * 5 Hz - 200 B/s of actual data. Writing each record as its own 512-byte
 * block meant issuing 2560 B/s of block programs: 12.8x write
 * amplification, one full program/erase cycle per 40 bytes of payload,
 * and five exposures per second to the SD card's 250 ms worst-case busy
 * time.
 *
 * Records now accumulate into a 512-byte staging block and go out once it
 * fills - roughly every 2.4 s - or on an explicit flush.
 *
 * Layout: a 16-byte header so a block is self-describing when pulled off
 * the card offline, then as many whole records as fit.
 *
 *   (512 - 16) / 40 = 12.4  ->  12 records, 480 bytes, 16 bytes padding
 */
#define LOG_BLOCK_MAGIC          0x54424C31UL   /* "TBL1" */
#define LOG_RECORDS_PER_BLOCK    12U

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t seq;              /* block sequence number, from 0        */
    uint16_t record_count;     /* valid records in this block          */
    uint16_t record_size;      /* sizeof(LogRecord), for offline parse */
    uint32_t first_timestamp;  /* tick of the first record             */
} LogBlockHeader;

/*
 * What travels on logQueue.
 *
 * A flush is a queue item rather than a task notification so it stays in
 * order with the records around it: a flush posted after N records
 * flushes exactly those N, with no race against the staging buffer.
 */
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
