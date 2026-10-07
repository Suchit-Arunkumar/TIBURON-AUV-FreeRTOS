#include "logging_task.h"
#include "spi_owner_task.h"
#include "sd_logger.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

// Packs log records into 512-byte blocks and hands full blocks to
// spi_owner_task, the only task that touches SPI.

// A block leaves every ~1.4 s and an SD write takes at most 250 ms, so if
// the bus owner is still busy after 500 ms, drop the block rather than
// stall and lose the records after it.
#define SPI_POST_TIMEOUT_MS   500U

QueueHandle_t logQueue = NULL;

// static: 512 bytes is half this task's stack
static uint8_t  staging[SPI_BLOCK_BYTES];
static uint16_t staged_count = 0;
static uint32_t block_seq    = 0;
static uint32_t first_ts     = 0;

static volatile uint32_t blocks_emitted   = 0;
static volatile uint32_t records_staged   = 0;
static volatile uint32_t spi_post_drops   = 0;

uint32_t logging_blocks_emitted(void)  { return blocks_emitted; }
uint32_t logging_records_staged(void)  { return records_staged; }
uint32_t logging_spi_post_drops(void)  { return spi_post_drops; }

static void staging_reset(void)
{
    memset(staging, 0, sizeof(staging));
    staged_count = 0;
    first_ts     = 0;
}

// Add the header and send the block. A partial block is fine:
// record_count says how many records are real.
static void staging_flush(void)
{
    if (staged_count == 0U)
    {
        return;   /* nothing to write */
    }

    LogBlockHeader header;

    header.magic           = LOG_BLOCK_MAGIC;
    header.seq             = block_seq;
    header.record_count    = staged_count;
    header.record_size     = (uint16_t)sizeof(LogRecord);
    header.first_timestamp = first_ts;

    memcpy(staging, &header, sizeof(header));

    // static: 520 bytes, too big for the stack
    static SpiRequest req;

    req.type       = SPI_REQ_SD_BLOCK;
    req.block_addr = sd_logger_block_for_seq(block_seq);
    memcpy(req.block, staging, SPI_BLOCK_BYTES);

    if (xQueueSend(spiRequestQueue,
                   &req,
                   pdMS_TO_TICKS(SPI_POST_TIMEOUT_MS)) == pdPASS)
    {
        blocks_emitted++;
        block_seq++;
    }
    else
    {
        // Drop it, but don't advance seq: on the card a gap in seq then
        // means exactly this.
        spi_post_drops++;
    }

    staging_reset();
}

static void staging_add(const LogRecord *record)
{
    if (staged_count >= LOG_RECORDS_PER_BLOCK)
    {
        staging_flush();
    }

    if (staged_count == 0U)
    {
        first_ts = record->timestamp_ms;
    }

    memcpy(
        &staging[sizeof(LogBlockHeader) + (staged_count * sizeof(LogRecord))],
        record,
        sizeof(LogRecord)
    );

    staged_count++;
    records_staged++;

    // send as soon as it's full, not when the next record arrives
    if (staged_count >= LOG_RECORDS_PER_BLOCK)
    {
        staging_flush();
    }
}

void logging_task(void *argument)
{
    (void)argument;

    LogQueueItem item;

    staging_reset();

    while (1)
    {
        if (xQueueReceive(logQueue, &item, portMAX_DELAY) == pdPASS)
        {
            if (item.kind == LOG_ITEM_FLUSH)
            {
                // disarm or failsafe: write what we have now
                staging_flush();
            }
            else
            {
                staging_add(&item.record);
            }
        }
    }
}
