#include "display_task.h"

#include "oled.h"
#include "sd_logger.h"
#include "control_loop.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/*
 * Status refresh cadence. Not tied to the log write rate — this task
 * redraws the OLED on this timeout even if no SPI requests arrive,
 * and also redraws right after servicing a request so the two never
 * fight over separate scheduling.
 */
#define DISPLAY_REFRESH_MS   500U

QueueHandle_t spiRequestQueue = NULL;


void display_task(void *argument)
{
    (void)argument;

    sd_logger_init();

    SpiRequest req;
    char line[21];

    while (1)
    {
        /*
         * Block on the SPI request queue with a timeout. This is
         * what makes the task both the request handler AND the
         * periodic refresher without needing two separate wake
         * sources.
         */
        if (xQueueReceive(
                spiRequestQueue,
                &req,
                pdMS_TO_TICKS(DISPLAY_REFRESH_MS)
            ) == pdPASS)
        {
            if (req.type == SPI_REQ_SD_LOG)
            {
                /*
                 * This call can block for an unbounded time (card
                 * busy-wait in sd_card.c). Accepted P8 tradeoff:
                 * display_task's own refresh cadence is not
                 * guaranteed while a log write is in flight.
                 */
                sd_logger_write(&req.record);
            }
        }

        /*
         * Periodic status refresh — armed/link state, independent
         * of whatever triggered this wake.
         */
        oled_clear();

        snprintf(
            line,
            sizeof(line),
            "ARM:%d LINK:%d",
            control_loop_get_armed() ? 1 : 0,
            control_loop_get_link()  ? 1 : 0
        );

        oled_draw_string(0, 0, line);
        oled_update();
    }
}
