#include "logging_task.h"
#include "display_task.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * logging_task does not touch SPI directly (bus-owner architecture —
 * see display_task.c). It only receives LogRecords assembled by
 * control_task and forwards them as a request for display_task to
 * execute.
 */
#define SPI_REQUEST_TIMEOUT_MS   50U

QueueHandle_t logQueue = NULL;


void logging_task(void *argument)
{
    (void)argument;

    LogRecord record;

    while (1)
    {
        if (xQueueReceive(logQueue, &record, portMAX_DELAY) == pdPASS)
        {
            SpiRequest req;

            req.type   = SPI_REQ_SD_LOG;
            req.record = record;

            /*
             * Bounded wait — if display_task is stuck mid SD write
             * for longer than this, drop the record rather than
             * let a slow bus owner back this task up indefinitely.
             */
            xQueueSend(
                spiRequestQueue,
                &req,
                pdMS_TO_TICKS(SPI_REQUEST_TIMEOUT_MS)
            );
        }
    }
}
