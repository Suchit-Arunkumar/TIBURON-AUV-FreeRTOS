#include "vn200_task.h"

#include "uart3.h"
#include "vn200.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sensor_status.h"
#include "console.h"
/*
 * Absence detection.
 *
 * The ISR notifies this task on every UART IDLE. With no sensor wired
 * there is no traffic and no notification ever arrives, so a
 * portMAX_DELAY wait would park the task forever - indistinguishable
 * from a crash. A bounded wait lets it notice, publish SENSOR_ABSENT and
 * carry on.
 *
 * ABSENT after this many consecutive timeouts from boot; FAULTED if data
 * had been flowing and then stopped. Nothing is published in either
 * state, so no stale value ever reaches the queue.
 */
#define VN200_WAIT_MS          500U
#define VN200_ABSENT_TIMEOUTS  6U     /* ~3 s of silence */



QueueHandle_t vn200Queue = NULL;


void vn200_task(void *argument)
{
    (void)argument;


    uint8_t rx_data[64];

    /* Enable USART3's interrupt now that a task context exists. */
    uart3_irq_enable();

    uint32_t silent_cycles = 0;
    uint8_t  announced     = 0;

    while (1)
    {
        /*
         * Sleep until the USART3 IDLE ISR notifies us, or the wait
         * expires. Bounded, not portMAX_DELAY - see the note above.
         */
        if (ulTaskNotifyTake(
                pdTRUE,
                pdMS_TO_TICKS(VN200_WAIT_MS)
            ) == 0U)
        {
            /* Nothing arrived within the window. */
            if (silent_cycles < VN200_ABSENT_TIMEOUTS)
            {
                silent_cycles++;
            }

            if (silent_cycles >= VN200_ABSENT_TIMEOUTS)
            {
                SensorState next = (g_vn200_state == SENSOR_OK)
                                 ? SENSOR_FAULTED : SENSOR_ABSENT;

                if (g_vn200_state != next)
                {
                    g_vn200_state = next;

                    /* Announce each transition once, not every cycle. */
                    console_printf("VN200: %s", sensor_state_str(next));
                }
            }

            continue;
        }

        silent_cycles = 0;


        /*
         * Drain everything UART3 has buffered, 64 bytes at a time, and
         * feed it to the packet assembler. Reading only one chunk per
         * wake-up left the rest waiting for the next interrupt, so a
         * backlog could build until the ring overflowed.
         */
        uint16_t received;

        while ((received = uart3_read(rx_data, sizeof(rx_data))) > 0U)
        {
            for (uint16_t i = 0U; i < received; i++)
            {
                vn200_feed_byte(rx_data[i]);
            }
        }


        /*
         * Get the newest complete valid VN-200
         * measurement.
         */
        VN200Data data;


        if (vn200_get_data(&data))
        {
            if (g_vn200_state != SENSOR_OK)
            {
                g_vn200_state = SENSOR_OK;

                if (announced == 0U)
                {
                    announced = 1U;
                    console_printf("VN200: ok");
                }
            }

            /*
             * Queue length = 1.
             *
             * Keep newest measurement.
             */
            if (vn200Queue != NULL)
            {
                xQueueOverwrite(
                    vn200Queue,
                    &data
                );
            }
        }
    }
}
