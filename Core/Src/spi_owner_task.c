#include "spi_owner_task.h"

#include "spi.h"
#include "oled.h"
#include "sd_card.h"
#include "control_loop.h"
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/*
 * OLED refresh cadence.
 *
 * This is the xQueueReceive timeout, not a separate timer: when no bus
 * request arrives within this window the task falls through and redraws.
 * One wake source serves both jobs, and a burst of SD traffic naturally
 * displaces refreshes rather than competing with them.
 */
#define DISPLAY_REFRESH_MS   500U

QueueHandle_t spiRequestQueue = NULL;

static uint32_t sd_write_count = 0;
static uint32_t sd_error_count = 0;

uint32_t spi_owner_sd_writes(void) { return sd_write_count; }
uint32_t spi_owner_sd_errors(void) { return sd_error_count; }

/*
 * Every bus access goes through one of these two handlers, and each one
 * sets the SCK rate it needs before touching a chip select. That is the
 * whole reason this is a bus owner rather than a mutex: BR is private
 * state here, instead of an obligation on every caller.
 */

static void handle_sd_block(const SpiRequest *req)
{
    spi_set_baud(SPI_BR_SD_DATA);

    SD_Status st = sd_write_block(req->block_addr, req->block);

    if (st == SD_OK)
    {
        sd_write_count++;
    }
    else
    {
        sd_error_count++;

        /* Rate-limited: one line per failure would flood the console if
         * the card is removed mid-run. */
        if (sd_error_count <= 3U)
        {
            console_printf("SD write err @blk %lu: %s",
                           (unsigned long)req->block_addr,
                           sd_status_str(st));
        }
    }
}

static void handle_oled_frame(void)
{
    char line[21];

    spi_set_baud(SPI_BR_OLED);

    oled_clear();

    snprintf(
        line,
        sizeof(line),
        "ARM:%d LINK:%d",
        control_loop_get_armed() ? 1 : 0,
        control_loop_get_link()  ? 1 : 0
    );

    oled_draw_string(0, 0, line);

    snprintf(
        line,
        sizeof(line),
        "BLK:%lu ERR:%lu",
        (unsigned long)sd_write_count,
        (unsigned long)sd_error_count
    );

    oled_draw_string(0, 1, line);

    oled_update();
}

void spi_owner_task(void *argument)
{
    (void)argument;

    /*
     * P9: static, not a stack local.
     *
     * SpiRequest carries a 512-byte block by value - the deliberate
     * choice that keeps block ownership out of the handoff between
     * logging_task and this task. On the stack it made this task's frame
     * 536 bytes and pushed worst-case usage to 93% of its 1 KB
     * allocation. As a static it costs the same 520 bytes in .bss, where
     * there are ~100 KB free, and the queue semantics are untouched.
     *
     * Safe because exactly one task ever executes this function and it
     * is not reentrant.
     */
    static SpiRequest req;

    while (1)
    {
        if (xQueueReceive(
                spiRequestQueue,
                &req,
                pdMS_TO_TICKS(DISPLAY_REFRESH_MS)
            ) == pdPASS)
        {
            switch (req.type)
            {
                case SPI_REQ_SD_BLOCK:
                    handle_sd_block(&req);
                    break;

                case SPI_REQ_OLED_FRAME:
                    handle_oled_frame();
                    break;

                default:
                    /* Unknown request type: ignore rather than fault. A
                     * new device on this bus adds a case here. */
                    break;
            }
        }
        else
        {
            /* Receive timed out - use it as the display refresh tick. */
            handle_oled_frame();
        }
    }
}
