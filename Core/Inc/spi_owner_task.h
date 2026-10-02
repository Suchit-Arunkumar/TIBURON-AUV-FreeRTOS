#ifndef SPI_OWNER_TASK_H
#define SPI_OWNER_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include <stdint.h>

/*
 * SPI2 bus owner.
 *
 * Both the SSD1306 and the SD card sit on SPI2, and neither oled.c nor
 * sd_card.c has any locking - they are raw polled register drivers with
 * no task awareness.
 *
 * DECISION: exactly one task may ever call spi_*, oled_* or sd_*
 * functions. Everything else posts a request here.
 *
 * A mutex was the alternative and was rejected on two grounds. First,
 * control_task never touches SPI, so priority inheritance - the only real
 * advantage a mutex has - protects nothing here. Second, the bus needs
 * three different SCK rates (351.6 kHz for SD identification, 5.625 MHz
 * for SD data, 5.625 MHz for the OLED); under a mutex every acquirer
 * becomes responsible for programming CR1.BR correctly on every
 * acquisition, and a missed one fails silently and intermittently. As the
 * bus owner, this task holds BR as private state and sets it from the
 * request type it just dequeued.
 *
 * The decision inverts if any of these become true:
 *   1. A task at priority >= 5 needs the bus directly. A priority-3 owner
 *      serving a priority-7 requester through a queue is unbounded
 *      priority inversion with no inheritance at all. This is the one to
 *      watch: a synchronous SD read from control_task would do it.
 *   2. Worst-case single operation drops below ~20 ms AND more than one
 *      task needs the bus - the dedicated task stops paying for itself.
 *   3. BR never has to change - one device, or two sharing a rate.
 *
 * PRIORITY 3, and that is the only value that works:
 *   - strictly ABOVE every task that sends it requests (logging at 1), so
 *     a requester cannot preempt the owner and pile up more work while a
 *     transaction is in flight;
 *   - strictly BELOW every task with a deadline (control 7,
 *     comms 5, sensors 4), so a 250 ms SD program cycle is preemptible by
 *     all of them.
 *
 * The OLED is a request TYPE, not this task's identity. Adding a third
 * device to the bus is a new enum value plus a case, not a restructure.
 */

typedef enum
{
    /* Write one pre-filled 512-byte block. Payload is the staging buffer
     * assembled by logging_task, not a single record. */
    SPI_REQ_SD_BLOCK = 0,

    /* Redraw the OLED from current vehicle state. Carries no payload -
     * the owner reads state through the control_loop getters. */
    SPI_REQ_OLED_FRAME = 1
} SpiRequestType;

#define SPI_BLOCK_BYTES   512U

typedef struct
{
    SpiRequestType type;

    /* Valid only for SPI_REQ_SD_BLOCK. */
    uint32_t block_addr;
    uint8_t  block[SPI_BLOCK_BYTES];
} SpiRequest;

extern QueueHandle_t spiRequestQueue;

void spi_owner_task(void *argument);

/* Counters, for the Phase 9 console report. */
uint32_t spi_owner_sd_writes(void);
uint32_t spi_owner_sd_errors(void);

#endif /* SPI_OWNER_TASK_H */
